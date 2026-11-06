package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RouteData.RECORD_SECONDS;
import static org.firstinspires.ftc.teamcode.RouteData.START_HEADING_DEGREES;
import static org.firstinspires.ftc.teamcode.RouteData.START_X_INCHES;
import static org.firstinspires.ftc.teamcode.RouteData.START_Y_INCHES;

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
 * Drive normally for up to 30 seconds (gamepad1 = driving, gamepad2 = mechanisms).
 *
 * Every loop this records WHERE the robot is (from the Pinpoint) plus what the
 * mechanisms are doing. Playback then drives to those positions instead of replaying
 * stick inputs, so wheel slip and battery changes no longer make the run drift.
 *
 * Place the robot at the start position (RouteData.START_*) before pressing PLAY.
 * Recording ends after 30 seconds, or when you press START on gamepad1.
 */
@TeleOp(name = "Record Route (30s)", group = "Record")
public class TeleRecord extends LinearOpMode {

    /** How the sticks feel while recording. This only affects how YOU drive, not the replay. */
    private static final double STRAFE_SENSITIVITY = 1.0;
    private static final double TURN_SENSITIVITY = 0.8;

    @Override
    public void runOpMode() {
        // The same localizer and drivetrain that your Pedro Constants / Tests use
        Localizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        Drivetrain drivetrain = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        Mechanisms mechanisms = new Mechanisms(hardwareMap);

        Pose startPose = new Pose(START_X_INCHES, START_Y_INCHES, Math.toRadians(START_HEADING_DEGREES));
        localizer.setPose(startPose);

        telemetry.addData("Status", "Place robot at the start position, then press PLAY");
        telemetry.addData("Start position", "x=%.1f y=%.1f heading=%.0f deg",
                START_X_INCHES, START_Y_INCHES, START_HEADING_DEGREES);
        telemetry.addData("Length", "%.0f s (START on gamepad1 ends early)", RECORD_SECONDS);
        telemetry.update();

        sleep(1000); // let the Pinpoint settle, like your Tests do
        waitForStart();

        localizer.setPose(startPose);
        localizer.update();

        RouteData.Recording recording = new RouteData.Recording();
        recording.batteryVoltage = Mechanisms.readBatteryVoltage(hardwareMap);
        recording.startXInches = startPose.x();
        recording.startYInches = startPose.y();
        recording.startHeadingRadians = startPose.heading();

        double[] mechanismState = Mechanisms.newEmptyState();
        ElapsedTime recordingTimer = new ElapsedTime();

        while (opModeIsActive() && recordingTimer.seconds() < RECORD_SECONDS) {
            double secondsSinceStart = recordingTimer.seconds();

            // Driving: same stick mapping as your Tests / TeleOp (robot-centric)
            drivetrain.drive(new DrivePowers(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x * STRAFE_SENSITIVITY,
                    -gamepad1.right_stick_x * TURN_SENSITIVITY), true);

            // Mechanisms: same controls as RandomBS3Conglomerate
            Mechanisms.updateStateFromGamepad(gamepad2, mechanismState);
            mechanisms.sendToHardware(mechanismState);

            // Where is the robot right now?
            localizer.update();
            Pose currentPose = localizer.pose();
            recording.frames.add(new RouteData.RecordedFrame(
                    secondsSinceStart, currentPose.x(), currentPose.y(), currentPose.heading(),
                    mechanismState));

            if (gamepad1.start) break;

            telemetry.addData("Recording", "%.1f / %.0f s", secondsSinceStart, RECORD_SECONDS);
            telemetry.addData("Position", "x=%.1f y=%.1f heading=%.0f deg",
                    currentPose.x(), currentPose.y(), Math.toDegrees(currentPose.heading()));
            telemetry.addData("Frames recorded", recording.frames.size());
            telemetry.update();

            sleep(10);
        }

        drivetrain.stop();
        mechanisms.stopAllMotion();

        recording.durationSeconds = recording.frames.isEmpty()
                ? 0
                : recording.frames.get(recording.frames.size() - 1).timeSeconds;
        saveRecording(recording);
    }

    private void saveRecording(RouteData.Recording recording) {
        try {
            RouteData.save(recording);
            telemetry.addData("Saved", "%d frames (%.1f s) to %s",
                    recording.frames.size(), recording.durationSeconds, RouteData.FILE_PATH);
        } catch (IOException e) {
            telemetry.addData("ERROR", "Could not save: " + e.getMessage());
        }
        telemetry.update();
        sleep(2000);
    }
}
