package org.firstinspires.ftc.teamcode;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.List;
import java.util.Locale;

/**
 * Everything about a recorded route that does NOT touch robot hardware:
 *  - the file format (one row per loop: time, x, y, heading, mechanism outputs)
 *  - the planner that turns the raw recording into a short list of RouteSteps
 *
 * A RouteStep is either
 *   DRIVE A LINE: go in a straight line from one recorded position to the next, or
 *   HOLD:         stay at a position (turning in place, or waiting while the
 *                 mechanisms run with the same timing you used when you drove it).
 *
 * All positions are in inches and all angles in radians, in the Pedro field frame,
 * exactly as reported by the Pinpoint localizer.
 */
public final class RouteData {
    private RouteData() {}

    // ------------------------------------------------------------------
    // Settings shared by TeleRecord and Playback
    // ------------------------------------------------------------------
    public static final String FILE_PATH = "/sdcard/FIRST/recorded_route.csv";
    public static final double RECORD_SECONDS = 30.0;

    /** Where the robot must be placed for BOTH recording and playback. */
    public static final double START_X_INCHES = 0.0;
    public static final double START_Y_INCHES = 0.0;
    public static final double START_HEADING_DEGREES = 0.0;

    // ------------------------------------------------------------------
    // Mechanism state layout: one number per output, stored in an array.
    // Entries 0-4 are motor / continuous-servo powers, entries 5-8 are servo positions.
    // A servo position of NaN means "never commanded" and is left alone.
    // ------------------------------------------------------------------
    public static final int LEFT_SHOOTER = 0;
    public static final int RIGHT_SHOOTER = 1;
    public static final int INTAKE = 2;
    public static final int STOPPER = 3;
    public static final int STOPPER_PARTNER = 4; // the "ThisOne2" continuous servo
    public static final int ANGLE_SERVO_1 = 5;
    public static final int ANGLE_SERVO_2 = 6;
    public static final int FLOWER_RIGHT = 7;
    public static final int FLOWER_LEFT = 8;
    public static final int MECHANISM_COUNT = 9;

    private static final int LAST_POWER_OUTPUT = STOPPER_PARTNER;
    private static final int FILE_COLUMNS = 4 + MECHANISM_COUNT; // time, x, y, heading + mechanisms
    private static final String FILE_TAG = "ODO1";

    // ------------------------------------------------------------------
    // Planner tuning
    // ------------------------------------------------------------------
    /** How closely the replayed path must follow the path you drove. */
    public static final double PATH_TOLERANCE_INCHES = 1.5;
    /** A heading error this big counts as much as PATH_TOLERANCE_INCHES of position error. */
    public static final double HEADING_TOLERANCE_DEGREES = 8.0;
    /** One straight line never changes the robot's heading by more than this. */
    public static final double MAX_HEADING_CHANGE_PER_LINE_DEGREES = 70.0;
    /** Moves shorter than this are not worth driving. */
    public static final double MIN_LINE_LENGTH_INCHES = 1.0;
    /** Heading changes smaller than this (with no movement) are ignored. */
    public static final double MIN_TURN_DEGREES = 4.0;
    /** The robot counts as "stopped" if it moves less than this ... */
    public static final double STILL_DISTANCE_INCHES = 0.75;
    public static final double STILL_ANGLE_DEGREES = 3.0;
    /** ... for this long. */
    public static final double STILL_WINDOW_SECONDS = 0.30;
    /** A stop only becomes a timed hold if it lasts this long AND a mechanism was active. */
    public static final double MIN_STOP_SECONDS = 0.40;

    private static final double HEADING_TOLERANCE_RADIANS = Math.toRadians(HEADING_TOLERANCE_DEGREES);
    private static final double MAX_HEADING_CHANGE_PER_LINE_RADIANS =
            Math.toRadians(MAX_HEADING_CHANGE_PER_LINE_DEGREES);
    private static final double MIN_TURN_RADIANS = Math.toRadians(MIN_TURN_DEGREES);
    private static final double STILL_ANGLE_RADIANS = Math.toRadians(STILL_ANGLE_DEGREES);

    // ------------------------------------------------------------------
    // Data classes
    // ------------------------------------------------------------------

    /** One snapshot taken during recording: where the robot was and what the mechanisms did. */
    public static final class RecordedFrame {
        public final double timeSeconds;
        public final double xInches, yInches, headingRadians;
        public final double[] mechanismState;

        public RecordedFrame(double timeSeconds, double xInches, double yInches,
                             double headingRadians, double[] mechanismState) {
            this.timeSeconds = timeSeconds;
            this.xInches = xInches;
            this.yInches = yInches;
            this.headingRadians = headingRadians;
            this.mechanismState = mechanismState.clone();
        }
    }

    /** A whole recording: header information plus every frame. */
    public static final class Recording {
        public double batteryVoltage = 12.0;
        public double durationSeconds = 0;
        public double startXInches, startYInches, startHeadingRadians;
        public final List<RecordedFrame> frames = new ArrayList<>();
    }

    /** A mechanism state that should be applied some time after a hold begins. */
    public static final class TimedMechanismState {
        public final double secondsIntoHold;
        public final double[] mechanismState;

        TimedMechanismState(double secondsIntoHold, double[] mechanismState) {
            this.secondsIntoHold = secondsIntoHold;
            this.mechanismState = mechanismState.clone();
        }
    }

    /** One instruction for Playback: drive a line, or hold a position. */
    public static final class RouteStep {
        /** false = drive a straight line, true = hold a position. */
        public final boolean isHold;

        /** Drive-a-line steps only: where the line starts. */
        public final double startX, startY, startHeading;

        /** Where the robot should end up (end of the line, or the position to hold). */
        public final double targetX, targetY, targetHeading;

        /** Mechanism state applied when this step begins. */
        public final double[] mechanismStateAtStart;

        /** Hold steps only: how long to stay (0 means "turn in place until settled"). */
        public final double holdSeconds;

        /** Hold steps only: mechanism changes, by time since the hold began. */
        public final List<TimedMechanismState> mechanismTimeline;

        private RouteStep(boolean isHold,
                          double startX, double startY, double startHeading,
                          double targetX, double targetY, double targetHeading,
                          double[] mechanismStateAtStart, double holdSeconds,
                          List<TimedMechanismState> mechanismTimeline) {
            this.isHold = isHold;
            this.startX = startX;
            this.startY = startY;
            this.startHeading = startHeading;
            this.targetX = targetX;
            this.targetY = targetY;
            this.targetHeading = normalizeAngle(targetHeading);
            this.mechanismStateAtStart = mechanismStateAtStart.clone();
            this.holdSeconds = holdSeconds;
            this.mechanismTimeline = mechanismTimeline;
        }

        static RouteStep driveLine(double startX, double startY, double startHeading,
                                   double targetX, double targetY, double targetHeading,
                                   double[] mechanismState) {
            return new RouteStep(false, startX, startY, normalizeAngle(startHeading),
                    targetX, targetY, targetHeading,
                    mechanismState, 0, new ArrayList<TimedMechanismState>());
        }

        static RouteStep holdPosition(double x, double y, double heading,
                                      double[] mechanismState, double seconds,
                                      List<TimedMechanismState> mechanismTimeline) {
            return new RouteStep(true, x, y, heading, x, y, heading,
                    mechanismState, seconds, mechanismTimeline);
        }

        /** Drive-a-line steps: straight-line length in inches. */
        public double lengthInches() {
            return Math.hypot(targetX - startX, targetY - startY);
        }

        /** Give up on a step that takes far longer than a normal drive would. */
        public double timeoutSeconds() {
            return isHold ? holdSeconds + 2.0 : 1.5 + lengthInches() / 12.0;
        }

        @Override
        public String toString() {
            if (isHold) {
                return String.format(Locale.US, "HOLD (%.1f, %.1f, %.0f deg) for %.2f s, %d mechanism change(s)",
                        targetX, targetY, Math.toDegrees(targetHeading),
                        holdSeconds, mechanismTimeline.size());
            }
            return String.format(Locale.US, "DRIVE (%.1f, %.1f, %.0f deg) -> (%.1f, %.1f, %.0f deg)",
                    startX, startY, Math.toDegrees(startHeading),
                    targetX, targetY, Math.toDegrees(targetHeading));
        }
    }

    // ------------------------------------------------------------------
    // Angle helper
    // ------------------------------------------------------------------

    /** Wrap an angle into the range (-pi, pi]. */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ------------------------------------------------------------------
    // File reading and writing
    // ------------------------------------------------------------------
    public static void save(Recording recording) throws IOException {
        save(recording, FILE_PATH);
    }

    public static void save(Recording recording, String path) throws IOException {
        File file = new File(path);
        File folder = file.getParentFile();
        if (folder != null && !folder.exists()) folder.mkdirs();

        try (BufferedWriter writer = new BufferedWriter(new FileWriter(file))) {
            writer.write(String.format(Locale.US, "#%s,%.3f,%.3f,%.5f,%.5f,%.6f",
                    FILE_TAG, recording.batteryVoltage, recording.durationSeconds,
                    recording.startXInches, recording.startYInches, recording.startHeadingRadians));
            writer.newLine();

            for (RecordedFrame frame : recording.frames) {
                StringBuilder row = new StringBuilder();
                row.append(String.format(Locale.US, "%.4f,%.4f,%.4f,%.5f",
                        frame.timeSeconds, frame.xInches, frame.yInches, frame.headingRadians));
                for (double value : frame.mechanismState) {
                    row.append(',').append(String.format(Locale.US, "%.4f", value)); // NaN is written as "NaN"
                }
                writer.write(row.toString());
                writer.newLine();
            }
        }
    }

    public static Recording load() throws IOException {
        return load(FILE_PATH);
    }

    public static Recording load(String path) throws IOException {
        Recording recording = new Recording();
        boolean foundHeader = false;

        try (BufferedReader reader = new BufferedReader(new FileReader(path))) {
            String line;
            int lineNumber = 0;
            while ((line = reader.readLine()) != null) {
                lineNumber++;
                line = line.trim();
                if (line.isEmpty()) continue;

                try {
                    if (line.startsWith("#")) {
                        String[] header = line.substring(1).split(",");
                        if (header.length < 6 || !FILE_TAG.equals(header[0])) {
                            throw new IOException("Not a route recording (old format?). Record again.");
                        }
                        recording.batteryVoltage = Double.parseDouble(header[1]);
                        recording.durationSeconds = Double.parseDouble(header[2]);
                        recording.startXInches = Double.parseDouble(header[3]);
                        recording.startYInches = Double.parseDouble(header[4]);
                        recording.startHeadingRadians = Double.parseDouble(header[5]);
                        foundHeader = true;
                        continue;
                    }

                    if (!foundHeader) {
                        throw new IOException("Missing header (old format?). Record again.");
                    }

                    String[] parts = line.split(",");
                    if (parts.length != FILE_COLUMNS) {
                        throw new IOException("Line " + lineNumber + ": expected " + FILE_COLUMNS + " columns");
                    }
                    double[] numbers = new double[FILE_COLUMNS];
                    for (int column = 0; column < FILE_COLUMNS; column++) {
                        numbers[column] = Double.parseDouble(parts[column]); // understands "NaN"
                    }
                    recording.frames.add(new RecordedFrame(
                            numbers[0], numbers[1], numbers[2], numbers[3],
                            Arrays.copyOfRange(numbers, 4, FILE_COLUMNS)));
                } catch (NumberFormatException e) {
                    throw new IOException("Line " + lineNumber + ": bad number");
                }
            }
        }

        if (recording.frames.size() < 2) throw new IOException("Recording has too few frames");
        return recording;
    }

    // ------------------------------------------------------------------
    // Planner: raw recording -> short list of steps
    // ------------------------------------------------------------------
    public static List<RouteStep> planRoute(List<RecordedFrame> frames) {
        return new Planner(frames).buildSteps();
    }

    private static boolean sameValue(double a, double b) {
        if (Double.isNaN(a) || Double.isNaN(b)) return Double.isNaN(a) && Double.isNaN(b);
        return Math.abs(a - b) < 1e-9;
    }

    private static boolean sameMechanismState(double[] a, double[] b) {
        for (int i = 0; i < MECHANISM_COUNT; i++) {
            if (!sameValue(a[i], b[i])) return false;
        }
        return true;
    }

    /** A time the robot sat still while a mechanism was doing something. */
    private static final class Stop {
        final int firstFrame, lastFrame;              // the whole time the robot was still
        final int firstActiveFrame, lastActiveFrame;  // the part where a mechanism was active

        Stop(int firstFrame, int lastFrame, int firstActiveFrame, int lastActiveFrame) {
            this.firstFrame = firstFrame;
            this.lastFrame = lastFrame;
            this.firstActiveFrame = firstActiveFrame;
            this.lastActiveFrame = lastActiveFrame;
        }
    }

    private static final class Planner {
        final int frameCount;
        final double[] timeSeconds, xInches, yInches;
        final double[] headingRadians; // "unwrapped" so it never jumps by a full turn
        final double[][] mechanismStates;
        final List<RouteStep> steps = new ArrayList<>();

        Planner(List<RecordedFrame> frames) {
            frameCount = frames.size();
            timeSeconds = new double[frameCount];
            xInches = new double[frameCount];
            yInches = new double[frameCount];
            headingRadians = new double[frameCount];
            mechanismStates = new double[frameCount][];
            for (int i = 0; i < frameCount; i++) {
                RecordedFrame frame = frames.get(i);
                timeSeconds[i] = frame.timeSeconds;
                xInches[i] = frame.xInches;
                yInches[i] = frame.yInches;
                headingRadians[i] = (i == 0)
                        ? frame.headingRadians
                        : headingRadians[i - 1] + normalizeAngle(frame.headingRadians - headingRadians[i - 1]);
                mechanismStates[i] = frame.mechanismState;
            }
        }

        List<RouteStep> buildSteps() {
            if (frameCount < 2) return steps;

            List<Stop> stops = findStops(findStillFrames());
            int sectionStartFrame = 0;
            for (Stop stop : stops) {
                planDrivingSection(sectionStartFrame, stop.firstFrame);
                removeTurnBeforeHold(); // the hold itself turns to its position and waits to settle
                steps.add(makeTimedHold(stop));
                sectionStartFrame = stop.lastFrame;
            }
            planDrivingSection(sectionStartFrame, frameCount - 1);
            return steps;
        }

        // ---- finding the times the robot was stopped ----

        boolean[] findStillFrames() {
            boolean[] isStill = new boolean[frameCount];
            for (int frame = 0; frame < frameCount; frame++) {
                isStill[frame] = isStillOverWindow(frame, +1) || isStillOverWindow(frame, -1);
            }
            return isStill;
        }

        /**
         * True if, over a full STILL_WINDOW_SECONDS looking forward (direction +1) or
         * backward (direction -1) from this frame, the robot hardly moved.
         */
        boolean isStillOverWindow(int frame, int direction) {
            int windowEnd = frame;
            while (windowEnd + direction >= 0 && windowEnd + direction < frameCount
                    && Math.abs(timeSeconds[windowEnd] - timeSeconds[frame]) < STILL_WINDOW_SECONDS) {
                windowEnd += direction;
            }
            if (Math.abs(timeSeconds[windowEnd] - timeSeconds[frame]) < STILL_WINDOW_SECONDS) {
                return false; // not enough recording on this side to judge
            }
            for (int other = Math.min(frame, windowEnd); other <= Math.max(frame, windowEnd); other++) {
                double distanceMoved = Math.hypot(xInches[other] - xInches[frame],
                        yInches[other] - yInches[frame]);
                double angleTurned = Math.abs(headingRadians[other] - headingRadians[frame]);
                if (distanceMoved > STILL_DISTANCE_INCHES || angleTurned > STILL_ANGLE_RADIANS) {
                    return false;
                }
            }
            return true;
        }

        /** Stops that lasted long enough AND had a mechanism doing something. */
        List<Stop> findStops(boolean[] isStill) {
            List<Stop> stops = new ArrayList<>();
            int frame = 0;
            while (frame < frameCount) {
                if (!isStill[frame]) {
                    frame++;
                    continue;
                }
                int firstFrame = frame;
                while (frame + 1 < frameCount && isStill[frame + 1]) frame++;
                int lastFrame = frame;
                frame++;

                if (timeSeconds[lastFrame] - timeSeconds[firstFrame] < MIN_STOP_SECONDS) continue;
                int[] activeFrames = findActiveFrames(firstFrame, lastFrame);
                if (activeFrames != null) {
                    stops.add(new Stop(firstFrame, lastFrame, activeFrames[0], activeFrames[1]));
                }
            }
            return stops;
        }

        /**
         * First and last frame between firstFrame and lastFrame where a mechanism was running
         * or changed, or null if nothing happened. Idle time before and after is not worth
         * waiting for.
         */
        int[] findActiveFrames(int firstFrame, int lastFrame) {
            int firstActive = -1;
            int lastActive = -1;
            for (int frame = firstFrame; frame <= lastFrame; frame++) {
                boolean isActive = false;
                for (int output = 0; output <= LAST_POWER_OUTPUT; output++) {
                    if (Math.abs(mechanismStates[frame][output]) > 1e-6) isActive = true;
                }
                if (frame > 0 && !sameMechanismState(mechanismStates[frame], mechanismStates[frame - 1])) {
                    isActive = true;
                }
                if (isActive) {
                    if (firstActive < 0) firstActive = frame;
                    lastActive = frame;
                }
            }
            return firstActive < 0 ? null : new int[]{firstActive, lastActive};
        }

        /** A hold at the average position of the stop, replaying only the active part of it. */
        RouteStep makeTimedHold(Stop stop) {
            double sumX = 0, sumY = 0, sumHeading = 0;
            for (int frame = stop.firstFrame; frame <= stop.lastFrame; frame++) {
                sumX += xInches[frame];
                sumY += yInches[frame];
                sumHeading += headingRadians[frame];
            }
            int frameTotal = stop.lastFrame - stop.firstFrame + 1;

            List<TimedMechanismState> timeline = new ArrayList<>();
            double[] previousState = null;
            for (int frame = stop.firstActiveFrame; frame <= stop.lastActiveFrame; frame++) {
                if (previousState == null || !sameMechanismState(mechanismStates[frame], previousState)) {
                    double secondsIntoHold = timeSeconds[frame] - timeSeconds[stop.firstActiveFrame];
                    timeline.add(new TimedMechanismState(secondsIntoHold, mechanismStates[frame]));
                    previousState = mechanismStates[frame];
                }
            }

            // Never 0: a hold of 0 seconds means "turn in place".
            double holdSeconds = Math.max(0.05,
                    timeSeconds[stop.lastActiveFrame] - timeSeconds[stop.firstActiveFrame]);

            return RouteStep.holdPosition(sumX / frameTotal, sumY / frameTotal, sumHeading / frameTotal,
                    mechanismStates[stop.firstActiveFrame], holdSeconds, timeline);
        }

        void removeTurnBeforeHold() {
            int lastIndex = steps.size() - 1;
            if (lastIndex >= 0 && steps.get(lastIndex).isHold && steps.get(lastIndex).holdSeconds == 0) {
                steps.remove(lastIndex);
            }
        }

        // ---- turning the driving parts of the recording into lines and turns ----

        void planDrivingSection(int startFrame, int endFrame) {
            if (endFrame <= startFrame) return;

            // Every frame where a mechanism output changed must become a waypoint,
            // so Playback can trigger it when the robot reaches that spot.
            List<Integer> mustKeepFrames = new ArrayList<>();
            mustKeepFrames.add(startFrame);
            for (int frame = startFrame + 1; frame < endFrame; frame++) {
                if (!sameMechanismState(mechanismStates[frame], mechanismStates[frame - 1])) {
                    mustKeepFrames.add(frame);
                }
            }
            mustKeepFrames.add(endFrame);

            // Between the must-keep frames, keep only the points needed to follow the path.
            List<Integer> waypointFrames = new ArrayList<>();
            for (int i = 0; i + 1 < mustKeepFrames.size(); i++) {
                int fromFrame = mustKeepFrames.get(i);
                int toFrame = mustKeepFrames.get(i + 1);
                waypointFrames.add(fromFrame);
                addImportantPointsBetween(fromFrame, toFrame, waypointFrames);
            }
            waypointFrames.add(endFrame);

            // Turn consecutive waypoints into drive-a-line or turn-in-place steps.
            int currentFrame = waypointFrames.get(0);
            double[] currentMechanismState = mechanismStates[currentFrame];
            for (int i = 1; i < waypointFrames.size(); i++) {
                int nextFrame = waypointFrames.get(i);
                double distance = Math.hypot(xInches[nextFrame] - xInches[currentFrame],
                        yInches[nextFrame] - yInches[currentFrame]);
                double headingChange = Math.abs(headingRadians[nextFrame] - headingRadians[currentFrame]);

                if (distance >= MIN_LINE_LENGTH_INCHES) {
                    steps.add(RouteStep.driveLine(
                            xInches[currentFrame], yInches[currentFrame], headingRadians[currentFrame],
                            xInches[nextFrame], yInches[nextFrame], headingRadians[nextFrame],
                            currentMechanismState));
                    currentFrame = nextFrame;
                    currentMechanismState = mechanismStates[nextFrame];
                } else if (headingChange >= MIN_TURN_RADIANS) {
                    steps.add(RouteStep.holdPosition(
                            xInches[nextFrame], yInches[nextFrame], headingRadians[nextFrame],
                            currentMechanismState, 0.0, new ArrayList<TimedMechanismState>()));
                    currentFrame = nextFrame;
                    currentMechanismState = mechanismStates[nextFrame];
                } else {
                    // No real movement: just carry the mechanism state forward.
                    currentMechanismState = mechanismStates[nextFrame];
                }
            }
        }

        /**
         * Ramer-Douglas-Peucker line simplification: adds (in order) the frames strictly
         * between fromFrame and toFrame that are needed to stay within the path tolerance.
         */
        void addImportantPointsBetween(int fromFrame, int toFrame, List<Integer> output) {
            if (toFrame - fromFrame < 2) return;

            boolean[] keepFrame = new boolean[toFrame - fromFrame + 1];
            Deque<int[]> rangesToCheck = new ArrayDeque<>();
            rangesToCheck.push(new int[]{fromFrame, toFrame});

            while (!rangesToCheck.isEmpty()) {
                int[] range = rangesToCheck.pop();
                int rangeStart = range[0];
                int rangeEnd = range[1];
                if (rangeEnd - rangeStart < 2) continue;

                int farthestFrame = -1;
                double farthestDistance = -1;
                for (int frame = rangeStart + 1; frame < rangeEnd; frame++) {
                    double distance = distanceFromLine(rangeStart, rangeEnd, frame);
                    if (distance > farthestDistance) {
                        farthestDistance = distance;
                        farthestFrame = frame;
                    }
                }

                double lineLength = Math.hypot(xInches[rangeEnd] - xInches[rangeStart],
                        yInches[rangeEnd] - yInches[rangeStart]);
                boolean headingChangeTooLarge = lineLength >= MIN_LINE_LENGTH_INCHES
                        && Math.abs(headingRadians[rangeEnd] - headingRadians[rangeStart])
                        > MAX_HEADING_CHANGE_PER_LINE_RADIANS;
                if (farthestDistance <= PATH_TOLERANCE_INCHES && !headingChangeTooLarge) continue;

                if (farthestDistance < 1e-9) farthestFrame = (rangeStart + rangeEnd) / 2;
                keepFrame[farthestFrame - fromFrame] = true;
                rangesToCheck.push(new int[]{rangeStart, farthestFrame});
                rangesToCheck.push(new int[]{farthestFrame, rangeEnd});
            }

            for (int frame = fromFrame + 1; frame < toFrame; frame++) {
                if (keepFrame[frame - fromFrame]) output.add(frame);
            }
        }

        /**
         * How far a frame is from the straight line between two other frames, combining
         * position error (inches) and heading error (converted to inches).
         */
        double distanceFromLine(int lineStartFrame, int lineEndFrame, int frame) {
            double lineStartX = xInches[lineStartFrame];
            double lineStartY = yInches[lineStartFrame];
            double lineDx = xInches[lineEndFrame] - lineStartX;
            double lineDy = yInches[lineEndFrame] - lineStartY;
            double lineLengthSquared = lineDx * lineDx + lineDy * lineDy;

            // How far along the line (0 = start, 1 = end) is the closest point to this frame?
            double fractionAlongLine = 0;
            if (lineLengthSquared > 1e-9) {
                fractionAlongLine = ((xInches[frame] - lineStartX) * lineDx
                        + (yInches[frame] - lineStartY) * lineDy) / lineLengthSquared;
                fractionAlongLine = Math.max(0, Math.min(1, fractionAlongLine));
            }
            double sidewaysDistance = Math.hypot(
                    xInches[frame] - (lineStartX + fractionAlongLine * lineDx),
                    yInches[frame] - (lineStartY + fractionAlongLine * lineDy));

            // Heading only matters along a real line. If both ends are the same spot, the
            // robot is turning in place and only the final heading counts.
            double headingErrorInInches = 0;
            if (lineLengthSquared >= MIN_LINE_LENGTH_INCHES * MIN_LINE_LENGTH_INCHES) {
                double expectedHeading = headingRadians[lineStartFrame] + fractionAlongLine
                        * (headingRadians[lineEndFrame] - headingRadians[lineStartFrame]);
                headingErrorInInches = Math.abs(headingRadians[frame] - expectedHeading)
                        * (PATH_TOLERANCE_INCHES / HEADING_TOLERANCE_RADIANS);
            }
            return Math.max(sidewaysDistance, headingErrorInInches);
        }
    }
}
