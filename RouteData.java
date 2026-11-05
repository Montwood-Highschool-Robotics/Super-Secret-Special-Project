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
 *  - the planner that turns the raw log into a short list of Steps
 *
 * A Step is either
 *   MOVE: drive a straight line from one recorded pose to the next, or
 *   HOLD: stay at a pose (turning in place, or waiting while mechanisms run
 *         on the same timing you used when you drove it).
 *
 * Poses are in inches / radians in the Pedro field frame, exactly as reported
 * by the Pinpoint localizer.
 */
public final class RouteData {
    private RouteData() {}

    // ------------------------------------------------------------------
    // Settings shared by TeleRecord and Playback
    // ------------------------------------------------------------------
    public static final String FILE_PATH = "/sdcard/FIRST/recorded_route.csv";
    public static final double RECORD_SECONDS = 30.0;

    /** Where the robot must be placed for BOTH recording and playback. */
    public static final double START_X = 0.0;
    public static final double START_Y = 0.0;
    public static final double START_HEADING_DEG = 0.0;

    // ------------------------------------------------------------------
    // Mechanism state layout (one double per output)
    // 0-4 are motor / continuous-servo powers, 5-8 are servo positions.
    // A servo position of NaN means "never commanded" and is left alone.
    // ------------------------------------------------------------------
    public static final int L_SHOOT = 0, R_SHOOT = 1, INTAKE = 2, STOPPER = 3, THIS_ONE_2 = 4;
    public static final int ANGLE1 = 5, ANGLE2 = 6, FLOWER_R = 7, FLOWER_L = 8;
    public static final int MECH_COUNT = 9;
    private static final int LAST_POWER_INDEX = THIS_ONE_2;
    private static final int COLUMNS = 4 + MECH_COUNT;
    private static final String MAGIC = "ODO1";

    // ------------------------------------------------------------------
    // Planner tuning
    // ------------------------------------------------------------------
    /** How closely the replayed path must follow the path you drove (inches). */
    public static final double PATH_TOLERANCE_IN = 1.5;
    /** Heading error that counts as much as PATH_TOLERANCE_IN of position error. */
    public static final double HEADING_TOLERANCE_DEG = 8.0;
    /** A single straight segment never changes heading by more than this. */
    public static final double MAX_STEP_HEADING_DEG = 70.0;
    /** Moves shorter than this are not worth a path. */
    public static final double MIN_MOVE_IN = 1.0;
    /** Heading changes smaller than this (with no movement) are ignored. */
    public static final double MIN_TURN_DEG = 4.0;
    /** The robot counts as "stopped" if it moves less than this ... */
    public static final double STILL_DIST_IN = 0.75;
    public static final double STILL_ANGLE_DEG = 3.0;
    /** ... for this long. */
    public static final double STILL_WINDOW_S = 0.30;
    /** A stop only becomes a timed HOLD if it lasts this long and something was running. */
    public static final double MIN_DWELL_S = 0.40;

    private static final double HEADING_TOL_RAD = Math.toRadians(HEADING_TOLERANCE_DEG);
    private static final double MAX_STEP_HEADING_RAD = Math.toRadians(MAX_STEP_HEADING_DEG);
    private static final double MIN_TURN_RAD = Math.toRadians(MIN_TURN_DEG);
    private static final double STILL_ANGLE_RAD = Math.toRadians(STILL_ANGLE_DEG);

    // ------------------------------------------------------------------
    // Data classes
    // ------------------------------------------------------------------
    public static final class Frame {
        public final double t, x, y, h;
        public final double[] mech;

        public Frame(double t, double x, double y, double h, double[] mech) {
            this.t = t;
            this.x = x;
            this.y = y;
            this.h = h;
            this.mech = mech.clone();
        }
    }

    public static final class Recording {
        public double voltage = 12.0;
        public double duration = 0;
        public double startX, startY, startH;
        public final List<Frame> frames = new ArrayList<>();
    }

    public static final class TimedMech {
        public final double offset;
        public final double[] mech;

        TimedMech(double offset, double[] mech) {
            this.offset = offset;
            this.mech = mech.clone();
        }
    }

    public static final class Step {
        /** false = MOVE along a line, true = HOLD a pose. */
        public final boolean hold;
        /** MOVE only: where the line starts. */
        public final double fx, fy, fh;
        /** Target pose (end of the line, or the pose to hold). */
        public final double x, y, h;
        /** Mechanism state applied when the step starts. */
        public final double[] mech;
        /** HOLD only: how long to stay (0 = turn in place, wait until settled). */
        public final double holdSeconds;
        /** HOLD only: mechanism changes by time since the hold began. */
        public final List<TimedMech> timeline;

        private Step(boolean hold, double fx, double fy, double fh, double x, double y, double h,
                     double[] mech, double holdSeconds, List<TimedMech> timeline) {
            this.hold = hold;
            this.fx = fx;
            this.fy = fy;
            this.fh = fh;
            this.x = x;
            this.y = y;
            this.h = normalize(h);
            this.mech = mech.clone();
            this.holdSeconds = holdSeconds;
            this.timeline = timeline;
        }

        static Step move(double fx, double fy, double fh, double x, double y, double h, double[] mech) {
            return new Step(false, fx, fy, normalize(fh), x, y, h, mech, 0, new ArrayList<TimedMech>());
        }

        static Step hold(double x, double y, double h, double[] mech, double seconds,
                         List<TimedMech> timeline) {
            return new Step(true, x, y, h, x, y, h, mech, seconds, timeline);
        }

        /** MOVE: straight-line length in inches. */
        public double length() {
            return Math.hypot(x - fx, y - fy);
        }

        /** Give up on a step that takes far longer than a normal drive would. */
        public double timeoutSeconds() {
            return hold ? holdSeconds + 2.0 : 1.5 + length() / 12.0;
        }

        @Override
        public String toString() {
            if (hold) {
                return String.format(Locale.US, "HOLD (%.1f, %.1f, %.0f deg) for %.2f s, %d mech change(s)",
                        x, y, Math.toDegrees(h), holdSeconds, timeline.size());
            }
            return String.format(Locale.US, "MOVE (%.1f, %.1f, %.0f deg) -> (%.1f, %.1f, %.0f deg)",
                    fx, fy, Math.toDegrees(fh), x, y, Math.toDegrees(h));
        }
    }

    // ------------------------------------------------------------------
    // Angle helpers
    // ------------------------------------------------------------------
    /** Wrap an angle into (-pi, pi]. */
    public static double normalize(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }

    // ------------------------------------------------------------------
    // File I/O
    // ------------------------------------------------------------------
    public static void save(Recording r) throws IOException {
        save(r, FILE_PATH);
    }

    public static void save(Recording r, String path) throws IOException {
        File file = new File(path);
        File dir = file.getParentFile();
        if (dir != null && !dir.exists()) dir.mkdirs();

        try (BufferedWriter w = new BufferedWriter(new FileWriter(file))) {
            w.write(String.format(Locale.US, "#%s,%.3f,%.3f,%.5f,%.5f,%.6f",
                    MAGIC, r.voltage, r.duration, r.startX, r.startY, r.startH));
            w.newLine();
            for (Frame f : r.frames) {
                StringBuilder sb = new StringBuilder();
                sb.append(String.format(Locale.US, "%.4f,%.4f,%.4f,%.5f", f.t, f.x, f.y, f.h));
                for (double v : f.mech) {
                    sb.append(',').append(String.format(Locale.US, "%.4f", v)); // NaN -> "NaN"
                }
                w.write(sb.toString());
                w.newLine();
            }
        }
    }

    public static Recording load() throws IOException {
        return load(FILE_PATH);
    }

    public static Recording load(String path) throws IOException {
        Recording r = new Recording();
        boolean sawHeader = false;
        try (BufferedReader br = new BufferedReader(new FileReader(path))) {
            String line;
            int lineNo = 0;
            while ((line = br.readLine()) != null) {
                lineNo++;
                line = line.trim();
                if (line.isEmpty()) continue;
                try {
                    if (line.startsWith("#")) {
                        String[] h = line.substring(1).split(",");
                        if (h.length < 6 || !MAGIC.equals(h[0])) {
                            throw new IOException("Not a route recording (old format?). Record again.");
                        }
                        r.voltage = Double.parseDouble(h[1]);
                        r.duration = Double.parseDouble(h[2]);
                        r.startX = Double.parseDouble(h[3]);
                        r.startY = Double.parseDouble(h[4]);
                        r.startH = Double.parseDouble(h[5]);
                        sawHeader = true;
                        continue;
                    }
                    if (!sawHeader) {
                        throw new IOException("Missing header (old format?). Record again.");
                    }
                    String[] p = line.split(",");
                    if (p.length != COLUMNS) {
                        throw new IOException("Line " + lineNo + ": expected " + COLUMNS + " columns");
                    }
                    double[] v = new double[COLUMNS];
                    for (int i = 0; i < COLUMNS; i++) v[i] = Double.parseDouble(p[i]); // parses "NaN"
                    r.frames.add(new Frame(v[0], v[1], v[2], v[3], Arrays.copyOfRange(v, 4, COLUMNS)));
                } catch (NumberFormatException e) {
                    throw new IOException("Line " + lineNo + ": bad number");
                }
            }
        }
        if (r.frames.size() < 2) throw new IOException("Recording has too few frames");
        return r;
    }

    // ------------------------------------------------------------------
    // Planner
    // ------------------------------------------------------------------
    public static List<Step> plan(List<Frame> frames) {
        return new Planner(frames).run();
    }

    static boolean same(double a, double b) {
        if (Double.isNaN(a) || Double.isNaN(b)) return Double.isNaN(a) && Double.isNaN(b);
        return Math.abs(a - b) < 1e-9;
    }

    static boolean sameMech(double[] a, double[] b) {
        for (int i = 0; i < MECH_COUNT; i++) if (!same(a[i], b[i])) return false;
        return true;
    }

    private static final class Planner {
        final int n;
        final double[] t, x, y, h; // h is unwrapped so it never jumps by 2*pi
        final double[][] m;
        final List<Step> steps = new ArrayList<>();

        Planner(List<Frame> frames) {
            n = frames.size();
            t = new double[n];
            x = new double[n];
            y = new double[n];
            h = new double[n];
            m = new double[n][];
            for (int i = 0; i < n; i++) {
                Frame f = frames.get(i);
                t[i] = f.t;
                x[i] = f.x;
                y[i] = f.y;
                h[i] = i == 0 ? f.h : h[i - 1] + normalize(f.h - h[i - 1]);
                m[i] = f.mech;
            }
        }

        List<Step> run() {
            if (n < 2) return steps;

            List<int[]> dwells = findDwells(computeStill());
            int cursor = 0;
            for (int[] d : dwells) {
                planStretch(cursor, d[0]);
                dropTrailingTurn(); // the hold itself turns to its pose and waits for it to settle
                steps.add(makeDwell(d[0], d[1], d[2], d[3]));
                cursor = d[1];
            }
            planStretch(cursor, n - 1);
            return steps;
        }

        // ---- finding the moments the robot was stopped ----
        boolean[] computeStill() {
            boolean[] still = new boolean[n];
            for (int i = 0; i < n; i++) still[i] = windowStill(i, +1) || windowStill(i, -1);
            return still;
        }

        /** True if, over a full STILL_WINDOW_S in one direction, the robot hardly moved. */
        boolean windowStill(int i, int dir) {
            int j = i;
            while (j + dir >= 0 && j + dir < n && Math.abs(t[j] - t[i]) < STILL_WINDOW_S) j += dir;
            if (Math.abs(t[j] - t[i]) < STILL_WINDOW_S) return false; // not enough data this side
            for (int k = Math.min(i, j); k <= Math.max(i, j); k++) {
                if (Math.hypot(x[k] - x[i], y[k] - y[i]) > STILL_DIST_IN
                        || Math.abs(h[k] - h[i]) > STILL_ANGLE_RAD) {
                    return false;
                }
            }
            return true;
        }

        /** Stops that lasted long enough AND had something running: {start, end, firstActive, lastActive}. */
        List<int[]> findDwells(boolean[] still) {
            List<int[]> out = new ArrayList<>();
            int i = 0;
            while (i < n) {
                if (!still[i]) {
                    i++;
                    continue;
                }
                int a = i;
                while (i + 1 < n && still[i + 1]) i++;
                int b = i;
                i++;
                if (t[b] - t[a] < MIN_DWELL_S) continue;
                int[] span = activeSpan(a, b);
                if (span != null) out.add(new int[]{a, b, span[0], span[1]});
            }
            return out;
        }

        /**
         * First and last frame in [a, b] where a mechanism was running or changed, or null
         * if nothing happened. Idle time before and after is not worth waiting for.
         */
        int[] activeSpan(int a, int b) {
            int first = -1, last = -1;
            for (int i = a; i <= b; i++) {
                boolean active = false;
                for (int k = 0; k <= LAST_POWER_INDEX; k++) {
                    if (Math.abs(m[i][k]) > 1e-6) active = true;
                }
                if (i > 0 && !sameMech(m[i], m[i - 1])) active = true;
                if (active) {
                    if (first < 0) first = i;
                    last = i;
                }
            }
            return first < 0 ? null : new int[]{first, last};
        }

        /** A hold at the average pose of the stop, replaying only the active part of it. */
        Step makeDwell(int a, int b, int first, int last) {
            double sx = 0, sy = 0, sh = 0;
            for (int i = a; i <= b; i++) {
                sx += x[i];
                sy += y[i];
                sh += h[i];
            }
            int cnt = b - a + 1;

            List<TimedMech> timeline = new ArrayList<>();
            double[] prev = null;
            for (int i = first; i <= last; i++) {
                if (prev == null || !sameMech(m[i], prev)) {
                    timeline.add(new TimedMech(t[i] - t[first], m[i]));
                    prev = m[i];
                }
            }
            // never 0: a hold of 0 seconds means "turn in place"
            double seconds = Math.max(0.05, t[last] - t[first]);
            return Step.hold(sx / cnt, sy / cnt, sh / cnt, m[first], seconds, timeline);
        }

        void dropTrailingTurn() {
            int last = steps.size() - 1;
            if (last >= 0 && steps.get(last).hold && steps.get(last).holdSeconds == 0) steps.remove(last);
        }

        // ---- turning the moving part of the log into lines and turns ----
        void planStretch(int s, int e) {
            if (e <= s) return;

            // Every frame where a mechanism output changed must become a waypoint,
            // so Playback can trigger it when the robot reaches that spot.
            List<Integer> forced = new ArrayList<>();
            forced.add(s);
            for (int i = s + 1; i < e; i++) {
                if (!sameMech(m[i], m[i - 1])) forced.add(i);
            }
            forced.add(e);

            List<Integer> anchors = new ArrayList<>();
            for (int k = 0; k + 1 < forced.size(); k++) {
                int p = forced.get(k), q = forced.get(k + 1);
                anchors.add(p);
                simplify(p, q, anchors);
            }
            anchors.add(e);

            int cur = anchors.get(0);
            double[] curMech = m[cur];
            for (int k = 1; k < anchors.size(); k++) {
                int b = anchors.get(k);
                double dist = Math.hypot(x[b] - x[cur], y[b] - y[cur]);
                double dh = Math.abs(h[b] - h[cur]);
                if (dist >= MIN_MOVE_IN) {
                    steps.add(Step.move(x[cur], y[cur], h[cur], x[b], y[b], h[b], curMech));
                    cur = b;
                    curMech = m[b];
                } else if (dh >= MIN_TURN_RAD) {
                    steps.add(Step.hold(x[b], y[b], h[b], curMech, 0.0, new ArrayList<TimedMech>()));
                    cur = b;
                    curMech = m[b];
                } else {
                    curMech = m[b]; // no real movement: just carry the mechanism state forward
                }
            }
        }

        /** Ramer-Douglas-Peucker: keep only the points needed to stay within tolerance. */
        void simplify(int p, int q, List<Integer> out) {
            if (q - p < 2) return;
            boolean[] keep = new boolean[q - p + 1];
            Deque<int[]> stack = new ArrayDeque<>();
            stack.push(new int[]{p, q});
            while (!stack.isEmpty()) {
                int[] r = stack.pop();
                int lo = r[0], hi = r[1];
                if (hi - lo < 2) continue;

                int best = -1;
                double bestDev = -1;
                for (int i = lo + 1; i < hi; i++) {
                    double d = deviation(lo, hi, i);
                    if (d > bestDev) {
                        bestDev = d;
                        best = i;
                    }
                }
                double len = Math.hypot(x[hi] - x[lo], y[hi] - y[lo]);
                boolean headingTooBig = len >= MIN_MOVE_IN && Math.abs(h[hi] - h[lo]) > MAX_STEP_HEADING_RAD;
                if (bestDev <= PATH_TOLERANCE_IN && !headingTooBig) continue;

                if (bestDev < 1e-9) best = (lo + hi) / 2;
                keep[best - p] = true;
                stack.push(new int[]{lo, best});
                stack.push(new int[]{best, hi});
            }
            for (int i = p + 1; i < q; i++) if (keep[i - p]) out.add(i);
        }

        /** How far frame i is from the straight line lo->hi, mixing position and heading error. */
        double deviation(int lo, int hi, int i) {
            double ax = x[lo], ay = y[lo];
            double dx = x[hi] - ax, dy = y[hi] - ay;
            double len2 = dx * dx + dy * dy;

            double s = 0;
            if (len2 > 1e-9) {
                s = ((x[i] - ax) * dx + (y[i] - ay) * dy) / len2;
                s = Math.max(0, Math.min(1, s));
            }
            double perp = Math.hypot(x[i] - (ax + s * dx), y[i] - (ay + s * dy));

            // Heading only matters along a real line. If the endpoints are the same spot
            // the robot is turning in place and only the final heading counts.
            double headingTerm = 0;
            if (len2 >= MIN_MOVE_IN * MIN_MOVE_IN) {
                double expected = h[lo] + s * (h[hi] - h[lo]);
                headingTerm = Math.abs(h[i] - expected) * (PATH_TOLERANCE_IN / HEADING_TOL_RAD);
            }
            return Math.max(perp, headingTerm);
        }
    }
}
