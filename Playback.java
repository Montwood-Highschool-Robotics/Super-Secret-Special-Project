package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOps.TeleOpRecorderRevival;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;

/**
 * Replays the wheel powers recorded by TeleRecord, on the same timeline.
 * Powers are scaled by (recorded battery voltage / current battery voltage)
 * so a different battery charge doesn't change how far the robot travels.
 */
@Autonomous(name = "Playback Recorded Drive", group = "Record")
public class Playback extends LinearOpMode {

    private DcMotor FLW, BLW, FRW, BRW;

    private final ArrayList<double[]> frames = new ArrayList<>();
    private double recordedVoltage = 12.0;
    private double duration = 0;

    @Override
    public void runOpMode() {
        FLW = hardwareMap.dcMotor.get("FLW");
        BLW = hardwareMap.dcMotor.get("BLW");
        FRW = hardwareMap.dcMotor.get("FRW");
        BRW = hardwareMap.dcMotor.get("BRW");

        // Motor directions must match TeleRecord
        for (DcMotor m : new DcMotor[]{FLW, BLW, FRW, BRW}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        boolean loaded = loadRecording();
        telemetry.addData("Status", loaded
                ? "Loaded " + frames.size() + " frames (" + String.format("%.1f", duration) + " s)"
                : "NO RECORDING LOADED - run Record Drive first");
        telemetry.update();

        waitForStart();
        if (!loaded || frames.isEmpty()) return;

        double scale = recordedVoltage / getBatteryVoltage();
        scale = Math.max(0.8, Math.min(scale, 1.25)); // keep the correction sane

        ElapsedTime timer = new ElapsedTime();
        int index = 0;

        while (opModeIsActive() && timer.seconds() <= duration) {
            double t = timer.seconds();

            // Advance to the latest frame whose timestamp has passed
            while (index + 1 < frames.size() && frames.get(index + 1)[0] <= t) {
                index++;
            }

            double[] f = frames.get(index);
            FLW.setPower(clip(f[1] * scale));
            BLW.setPower(clip(f[2] * scale));
            FRW.setPower(clip(f[3] * scale));
            BRW.setPower(clip(f[4] * scale));

            telemetry.addData("Playback", "%.1f / %.1f s", t, duration);
            telemetry.addData("Frame", index + "/" + frames.size());
            telemetry.addData("Voltage scale", "%.2f", scale);
            telemetry.update();
        }

        FLW.setPower(0);
        BLW.setPower(0);
        FRW.setPower(0);
        BRW.setPower(0);
    }

    private boolean loadRecording() {
        try (BufferedReader r = new BufferedReader(new FileReader(TeleOpRecorderRevival.FILE_PATH))) {
            String line;
            while ((line = r.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) continue;
                if (line.startsWith("#")) {
                    String[] h = line.substring(1).split(",");
                    recordedVoltage = Double.parseDouble(h[0]);
                    duration = Double.parseDouble(h[1]);
                    continue;
                }
                String[] p = line.split(",");
                frames.add(new double[]{
                        Double.parseDouble(p[0]), Double.parseDouble(p[1]),
                        Double.parseDouble(p[2]), Double.parseDouble(p[3]),
                        Double.parseDouble(p[4])});
            }
            return !frames.isEmpty();
        } catch (Exception e) {
            telemetry.addData("Error", "Could not load file: " + e.getMessage());
            return false;
        }
    }

    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) result = Math.min(result, v);
        }
        return result == Double.POSITIVE_INFINITY ? 12.0 : result;
    }

    private static double clip(double p) {
        return Math.max(-1.0, Math.min(1.0, p));
    }
}
