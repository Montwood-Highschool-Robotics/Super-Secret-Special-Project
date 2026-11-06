package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.IOException;
import java.util.List;

/**
 * Replays a route recorded with "Record Route (30s)".
 *
 * Instead of repeating stick inputs, the robot drives to the POSITIONS you drove
 * through (using the Pinpoint and the Pedro follower), so it corrects for slip,
 * battery changes and bumps. Mechanisms fire when the robot reaches the spot where
 * you used them. When you stopped to shoot, the robot holds that spot, waits until
 * it is in position, and then replays your timing.
 *
 * Place the robot at the same start position you used when recording.
 */
@Autonomous(name = "Playback Recorded Route", group = "Record")
public class Playback extends LinearOpMode {

    /**
     * When one line leads straight into another, start the next line once the robot
     * is within this many inches of the end of the current one (so it doesn't stop at
     * every waypoint). Set to 0 to make it fully stop and settle at every waypoint.
     */
    private static final double START_NEXT_LINE_WITHIN_INCHES = 3.0;

    /** The robot counts as "in position" when it is this close to its target ... */
    private static final double IN_POSITION_TOLERANCE_INCHES = 1.0;
    private static final double IN_POSITION_TOLERANCE_DEGREES = 3.0;
    /** ... or after waiting this long, so one stubborn spot can't use up the whole autonomous. */
    private static final double IN_POSITION_TIMEOUT_SECONDS = 1.5;

    private Follower follower;
    private Mechanisms mechanisms;

    /** Time since the current step began. */
    private final ElapsedTime stepTimer = new ElapsedTime();
    /** Which entry of a hold's mechanism timeline was applied last (-1 = none yet). */
    private int timelineEntryIndex = -1;
    /** For timed holds: stepTimer reading when the robot got in position (-1 = not yet). */
    private double settledAtSeconds = -1;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        mechanisms = new Mechanisms(hardwareMap);

        RouteData.Recording recording;
        List<RouteData.RouteStep> routeSteps;
        try {
            recording = RouteData.load();
            routeSteps = RouteData.planRoute(recording.frames);
        } catch (IOException e) {
            telemetry.addData("ERROR", "Could not load route: " + e.getMessage());
            telemetry.addData("Fix", "Run \"Record Route (30s)\" first");
            telemetry.update();
            waitForStart();
            return;
        }

        Pose startPose = new Pose(recording.startXInches, recording.startYInches,
                recording.startHeadingRadians);
        follower.setPose(startPose);
        follower.update();

        telemetry.addData("Status", "Loaded %d frames -> %d steps",
                recording.frames.size(), routeSteps.size());
        telemetry.addData("Place robot at", "x=%.1f y=%.1f heading=%.0f deg",
                recording.startXInches, recording.startYInches,
                Math.toDegrees(recording.startHeadingRadians));
        for (int i = 0; i < Math.min(routeSteps.size(), 8); i++) {
            telemetry.addData("Step " + (i + 1), routeSteps.get(i).toString());
        }
        telemetry.update();

        waitForStart();
        if (routeSteps.isEmpty()) return;

        follower.setPose(startPose);
        follower.update();

        // Shooter power makes up for a different battery than when you recorded
        double batteryRatio = recording.batteryVoltage / Mechanisms.readBatteryVoltage(hardwareMap);
        mechanisms.setShooterPowerMultiplier(Math.max(0.8, Math.min(batteryRatio, 1.25)));

        int currentStepIndex = 0;
        beginStep(routeSteps.get(currentStepIndex));
        ElapsedTime telemetryTimer = new ElapsedTime();

        while (opModeIsActive() && currentStepIndex < routeSteps.size()) {
            follower.update(); // as often as possible, no sleep

            RouteData.RouteStep currentStep = routeSteps.get(currentStepIndex);
            RouteData.RouteStep nextStep = (currentStepIndex + 1 < routeSteps.size())
                    ? routeSteps.get(currentStepIndex + 1)
                    : null;

            if (isStepFinished(currentStep, nextStep)) {
                currentStepIndex++;
                if (currentStepIndex < routeSteps.size()) beginStep(routeSteps.get(currentStepIndex));
            }

            if (telemetryTimer.milliseconds() > 100) {
                Pose robotPose = follower.pose();
                telemetry.addData("Step", "%d / %d",
                        Math.min(currentStepIndex + 1, routeSteps.size()), routeSteps.size());
                telemetry.addData("Position", "x=%.1f y=%.1f heading=%.0f deg",
                        robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading()));
                if (currentStepIndex < routeSteps.size()) {
                    telemetry.addData("Doing", routeSteps.get(currentStepIndex).toString());
                }
                telemetry.update();
                telemetryTimer.reset();
            }
        }

        mechanisms.stopAllMotion();
    }

    /** Start a step: apply its mechanism state, then tell the follower where to go. */
    private void beginStep(RouteData.RouteStep step) {
        stepTimer.reset();
        timelineEntryIndex = -1;
        settledAtSeconds = -1;

        // A timed hold starts its mechanisms only once the robot is in position.
        boolean isTimedHold = step.isHold && step.holdSeconds > 0;
        if (!isTimedHold) mechanisms.sendToHardware(step.mechanismStateAtStart);

        if (step.isHold) {
            follower.hold(new Pose(step.targetX, step.targetY, step.targetHeading));
        } else {
            Pose lineStart = new Pose(step.startX, step.startY, step.startHeading);
            Pose lineEnd = new Pose(step.targetX, step.targetY, step.targetHeading);
            Path line = Paths.line(lineStart, lineEnd).linear(lineStart, lineEnd);
            follower.follow(line);
        }
    }

    private boolean isStepFinished(RouteData.RouteStep step, RouteData.RouteStep nextStep) {
        double secondsInStep = stepTimer.seconds();
        Pose robotPose = follower.pose();

        if (step.isHold) {
            double distanceToTarget = Math.hypot(robotPose.x() - step.targetX, robotPose.y() - step.targetY);
            double headingError = Math.abs(RouteData.normalizeAngle(robotPose.heading() - step.targetHeading));
            boolean isInPosition = distanceToTarget <= IN_POSITION_TOLERANCE_INCHES
                    && headingError <= Math.toRadians(IN_POSITION_TOLERANCE_DEGREES);
            boolean waitedTooLong = secondsInStep >= IN_POSITION_TIMEOUT_SECONDS;

            if (step.holdSeconds <= 0) {
                return isInPosition || waitedTooLong; // turn in place: done once in position
            }

            // Timed hold (for example, shooting): wait until in position, THEN replay your timing.
            if (settledAtSeconds < 0) {
                if (!(isInPosition || waitedTooLong)) return false;
                settledAtSeconds = secondsInStep;
            }
            double secondsSinceSettled = secondsInStep - settledAtSeconds;
            applyMechanismTimeline(step, secondsSinceSettled);
            return secondsSinceSettled >= step.holdSeconds;
        }

        // Driving a line
        boolean continueWithoutStopping = nextStep != null && !nextStep.isHold;
        double distanceToLineEnd = Math.hypot(robotPose.x() - step.targetX, robotPose.y() - step.targetY);
        if (continueWithoutStopping && START_NEXT_LINE_WITHIN_INCHES > 0
                && distanceToLineEnd <= START_NEXT_LINE_WITHIN_INCHES) {
            return true;
        }
        return follower.atParametricEnd() || secondsInStep >= step.timeoutSeconds();
    }

    /** During a hold, replay mechanism changes at the same times they happened in the recording. */
    private void applyMechanismTimeline(RouteData.RouteStep step, double secondsSinceSettled) {
        int latestEntryIndex = timelineEntryIndex;
        while (latestEntryIndex + 1 < step.mechanismTimeline.size()
                && step.mechanismTimeline.get(latestEntryIndex + 1).secondsIntoHold <= secondsSinceSettled) {
            latestEntryIndex++;
        }
        if (latestEntryIndex >= 0 && latestEntryIndex != timelineEntryIndex) {
            mechanisms.sendToHardware(step.mechanismTimeline.get(latestEntryIndex).mechanismState);
        }
        timelineEntryIndex = latestEntryIndex;
    }
}
