package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Locale;

/**
 * Drive normally for up to 30 seconds. Every loop the wheel powers are logged
 * with a timestamp. When the 30 s are up (or you press START on gamepad1)
 * the log is saved to a CSV file that the Playback OpMode replays.
 */
@TeleOp(name = "Record Drive (30s)", group = "Record")
public class TeleOpRecorderRevival extends LinearOpMode {

    public static final String FILE_PATH = "/sdcard/FIRST/recorded_drive.csv";
    public static final double RECORD_SECONDS = 30.0;

    private DcMotor FLW, BLW, FRW, BRW;

    // One row per loop: time (s) and the four wheel powers
    private final ArrayList<double[]> frames = new ArrayList<>();

    @Override
    public void runOpMode() {
        FLW = hardwareMap.dcMotor.get("FLW");
        BLW = hardwareMap.dcMotor.get("BLW");
        FRW = hardwareMap.dcMotor.get("FRW");
        BRW = hardwareMap.dcMotor.get("BRW");

        // If your robot needs reversed motors, set them here AND in Playback
        // (they must match), e.g. FLW.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{FLW, BLW, FRW, BRW}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        telemetry.addData("Status", "Ready. Press PLAY - recording starts immediately.");
        telemetry.addData("Length", "%.0f seconds (START button ends early)", RECORD_SECONDS);
        telemetry.update();

        waitForStart();

        double batteryVoltage = getBatteryVoltage();
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && timer.seconds() < RECORD_SECONDS) {
            double t = timer.seconds();

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x * 0.8;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            double fl = (y + x + rx) / denominator;
            double bl = (y - x + rx) / denominator;
            double fr = (y - x - rx) / denominator;
            double br = (y + x - rx) / denominator;

            FLW.setPower(fl);
            BLW.setPower(bl);
            FRW.setPower(fr);
            BRW.setPower(br);

            frames.add(new double[]{t, fl, bl, fr, br});

            if (gamepad1.start) break;

            telemetry.addData("Recording", "%.1f / %.0f s", t, RECORD_SECONDS);
            telemetry.addData("Frames", frames.size());
            telemetry.update();

            sleep(10);
        }

        stopDrive();
        saveRecording(batteryVoltage, frames.isEmpty() ? 0 : frames.get(frames.size() - 1)[0]);
    }

    private void stopDrive() {
        FLW.setPower(0);
        BLW.setPower(0);
        FRW.setPower(0);
        BRW.setPower(0);
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) result = Math.min(result, v);
        }
        return result == Double.POSITIVE_INFINITY ? 12.0 : result;
    }

    private void saveRecording(double voltage, double duration) {
        telemetry.addData("Status", "Saving...");
        telemetry.update();
        try {
            File file = new File(FILE_PATH);
            File dir = file.getParentFile();
            if (dir != null && !dir.exists()) dir.mkdirs();

            try (BufferedWriter w = new BufferedWriter(new FileWriter(file))) {
                // Header: voltage at record time, total duration
                w.write(String.format(Locale.US, "#%.3f,%.3f%n", voltage, duration));
                for (double[] f : frames) {
                    w.write(String.format(Locale.US, "%.4f,%.4f,%.4f,%.4f,%.4f%n",
                            f[0], f[1], f[2], f[3], f[4]));
                }
            }
            telemetry.addData("Success", "Saved %d frames (%.1f s)", frames.size(), duration);
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to save: " + e.getMessage());
        }
        telemetry.update();
        sleep(2000);
    }
}
