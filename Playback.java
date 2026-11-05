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
 * through (using the Pinpoint + Pedro follower), so it corrects for slip, battery
 * and bumps. Mechanisms fire when the robot reaches the spot where you used them,
 * and when you stopped to shoot, it holds that spot and replays your timing.
 *
 * Place the robot at the same start pose you used when recording.
 */
@Autonomous(name = "Playback Recorded Route", group = "Record")
public class Playback extends LinearOpMode {

    /**
     * On straight-through waypoints, move on to the next line when the robot is
     * within this many inches (so it doesn't stop at every waypoint).
     * Set to 0 to make it fully stop and settle at every waypoint.
     */
    private static final double ADVANCE_RADIUS_IN = 3.0;

    /** The robot counts as "in position" within these tolerances ... */
    private static final double SETTLE_POS_IN = 1.0;
    private static final double SETTLE_HEADING_DEG = 3.0;
    /** ... or after this long, so one stubborn spot can't eat the whole autonomous. */
    private static final double SETTLE_TIMEOUT_S = 1.5;

    private Follower follower;
    private Mechanisms mech;

    private final ElapsedTime stepTimer = new ElapsedTime();
    private int timelineIndex = -1;
    /** For timed holds: stepTimer reading when the robot settled (-1 = not yet). */
    private double settledAt = -1;

    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        mech = new Mechanisms(hardwareMap);

        RouteData.Recording rec;
        List<RouteData.Step> steps;
        try {
            rec = RouteData.load();
            steps = RouteData.plan(rec.frames);
        } catch (IOException e) {
            telemetry.addData("ERROR", "Could not load route: " + e.getMessage());
            telemetry.addData("Fix", "Run \"Record Route (30s)\" first");
            telemetry.update();
            waitForStart();
            return;
        }

        Pose startPose = new Pose(rec.startX, rec.startY, rec.startH);
        follower.setPose(startPose);
        follower.update();

        telemetry.addData("Status", "Loaded %d frames -> %d steps", rec.frames.size(), steps.size());
        telemetry.addData("Place robot at", "x=%.1f y=%.1f heading=%.0f deg",
                rec.startX, rec.startY, Math.toDegrees(rec.startH));
        for (int i = 0; i < Math.min(steps.size(), 8); i++) {
            telemetry.addData("Step " + (i + 1), steps.get(i).toString());
        }
        telemetry.update();

        waitForStart();
        if (steps.isEmpty()) return;

        follower.setPose(startPose);
        follower.update();

        // Shooter power compensates for a different battery than when you recorded
        double scale = rec.voltage / Mechanisms.batteryVoltage(hardwareMap);
        mech.setShooterScale(Math.max(0.8, Math.min(scale, 1.25)));

        int i = 0;
        beginStep(steps.get(i));
        ElapsedTime uiTimer = new ElapsedTime();

        while (opModeIsActive() && i < steps.size()) {
            follower.update(); // as often as possible, no sleep

            RouteData.Step current = steps.get(i);
            RouteData.Step next = i + 1 < steps.size() ? steps.get(i + 1) : null;

            if (isDone(current, next)) {
                i++;
                if (i < steps.size()) beginStep(steps.get(i));
            }

            if (uiTimer.milliseconds() > 100) {
                Pose p = follower.pose();
                telemetry.addData("Step", "%d / %d", Math.min(i + 1, steps.size()), steps.size());
                telemetry.addData("Pose", "x=%.1f y=%.1f h=%.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
                if (i < steps.size()) telemetry.addData("Doing", steps.get(i).toString());
                telemetry.update();
                uiTimer.reset();
            }
        }

        mech.stop();
    }

    /** Start a step: apply its mechanism state, then send the follower where it needs to go. */
    private void beginStep(RouteData.Step s) {
        stepTimer.reset();
        timelineIndex = -1;
        settledAt = -1;
        // A timed hold starts its mechanisms only once the robot is in position
        if (!(s.hold && s.holdSeconds > 0)) mech.apply(s.mech);

        if (s.hold) {
            follower.hold(new Pose(s.x, s.y, s.h));
        } else {
            Pose from = new Pose(s.fx, s.fy, s.fh);
            Pose to = new Pose(s.x, s.y, s.h);
            Path path = Paths.line(from, to).linear(from, to);
            follower.follow(path);
        }
    }

    private boolean isDone(RouteData.Step s, RouteData.Step next) {
        double elapsed = stepTimer.seconds();
        Pose p = follower.pose();

        if (s.hold) {
            boolean inPosition = Math.hypot(p.x() - s.x, p.y() - s.y) <= SETTLE_POS_IN
                    && Math.abs(RouteData.normalize(p.heading() - s.h)) <= Math.toRadians(SETTLE_HEADING_DEG);
            boolean gaveUp = elapsed >= SETTLE_TIMEOUT_S;

            if (s.holdSeconds <= 0) {
                return inPosition || gaveUp; // turn in place: done once settled
            }

            // Timed hold (e.g. shooting): wait until in position, THEN replay your timing
            if (settledAt < 0) {
                if (!(inPosition || gaveUp)) return false;
                settledAt = elapsed;
            }
            double sinceSettled = elapsed - settledAt;
            applyTimeline(s, sinceSettled);
            return sinceSettled >= s.holdSeconds;
        }

        // MOVE
        boolean keepRolling = next != null && !next.hold;
        double dist = Math.hypot(p.x() - s.x, p.y() - s.y);
        if (keepRolling && ADVANCE_RADIUS_IN > 0 && dist <= ADVANCE_RADIUS_IN) return true;
        return follower.atParametricEnd() || elapsed >= s.timeoutSeconds();
    }

    /** During a hold, replay mechanism changes at the same offsets they happened in the recording. */
    private void applyTimeline(RouteData.Step s, double elapsed) {
        int idx = timelineIndex;
        while (idx + 1 < s.timeline.size() && s.timeline.get(idx + 1).offset <= elapsed) idx++;
        if (idx >= 0 && idx != timelineIndex) mech.apply(s.timeline.get(idx).mech);
        timelineIndex = idx;
    }
}
