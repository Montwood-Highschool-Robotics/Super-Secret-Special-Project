package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RouteData.RECORD_SECONDS;
import static org.firstinspires.ftc.teamcode.RouteData.START_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.RouteData.START_X;
import static org.firstinspires.ftc.teamcode.RouteData.START_Y;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.IOException;

/**
 * Drive normally for up to 30 seconds (gamepad1 = drive, gamepad2 = mechanisms).
 *
 * Every loop this logs WHERE the robot is (Pinpoint pose) plus what the mechanisms
 * are doing. Playback then drives to those positions instead of replaying stick
 * inputs, so wheel slip and battery changes no longer make the run drift.
 *
 * Place the robot at the start pose (RouteData.START_*) before pressing PLAY.
 * The run ends after 30 s, or when you press START on gamepad1.
 */
@TeleOp(name = "Record Route (30s)", group = "Record")
public class TeleRecord extends LinearOpMode {

    /** Drive feel while recording. Only affects how YOU drive, not the replay. */
    private static final double STRAFE_SCALE = 1.0;
    private static final double TURN_SCALE = 0.8;

    @Override
    public void runOpMode() {
        // Same localizer + drivetrain your Pedro Constants / Tests use
        Localizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Drivetrain drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        Mechanisms mech = new Mechanisms(hardwareMap);

        Pose startPose = new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG));
        localizer.setPose(startPose);

        telemetry.addData("Status", "Place robot at the start pose, then press PLAY");
        telemetry.addData("Start pose", "x=%.1f y=%.1f heading=%.0f deg", START_X, START_Y, START_HEADING_DEG);
        telemetry.addData("Length", "%.0f s (START on gamepad1 ends early)", RECORD_SECONDS);
        telemetry.update();

        sleep(1000); // let the Pinpoint settle, like your Tests do
        waitForStart();

        localizer.setPose(startPose);
        localizer.update();

        RouteData.Recording rec = new RouteData.Recording();
        rec.voltage = Mechanisms.batteryVoltage(hardwareMap);
        rec.startX = startPose.x();
        rec.startY = startPose.y();
        rec.startH = startPose.heading();

        double[] mechState = Mechanisms.newState();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < RECORD_SECONDS) {
            double t = timer.seconds();

            // Drive: same stick mapping as your Tests / TeleOp (robot-centric)
            drivetrain.drive(new DrivePowers(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x * STRAFE_SCALE,
                    -gamepad1.right_stick_x * TURN_SCALE), true);

            // Mechanisms: same controls as RandomBS3Conglomerate
            Mechanisms.readGamepad(gamepad2, mechState);
            mech.apply(mechState);

            // Where are we?
            localizer.update();
            Pose p = localizer.pose();
            rec.frames.add(new RouteData.Frame(t, p.x(), p.y(), p.heading(), mechState));

            if (gamepad1.start) break;

            telemetry.addData("Recording", "%.1f / %.0f s", t, RECORD_SECONDS);
            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.0f deg", p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("Frames", rec.frames.size());
            telemetry.update();

            sleep(10);
        }

        drivetrain.stop();
        mech.stop();

        rec.duration = rec.frames.isEmpty() ? 0 : rec.frames.get(rec.frames.size() - 1).t;
        save(rec);
    }

    private void save(RouteData.Recording rec) {
        try {
            RouteData.save(rec);
            telemetry.addData("Saved", "%d frames (%.1f s) to %s",
                    rec.frames.size(), rec.duration, RouteData.FILE_PATH);
        } catch (IOException e) {
            telemetry.addData("ERROR", "Could not save: " + e.getMessage());
        }
        telemetry.update();
        sleep(2000);
    }
}
