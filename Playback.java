package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.TeleRecord.*;

/**
 * Replays everything recorded by TeleRecord (drive + mechanisms) on the same timeline.
 * Drive and shooter powers are scaled by (recorded battery voltage / current voltage)
 * so a different battery charge doesn't change the result as much.
 */
@Autonomous(name = "Playback Recorded Drive", group = "Record")
public class Playback extends LinearOpMode {

    private DcMotor FLW, BLW, FRW, BRW, LeftShoot, RightShoot, Intake;
    private Servo Angle1, Angle2, FlowerR, FlowerL;
    private CRServo Stopper;

    private final ArrayList<double[]> frames = new ArrayList<>();
    private double recordedVoltage = 12.0;
    private double duration = 0;

    @Override
    public void runOpMode() {
        FLW = hardwareMap.dcMotor.get("FLW");
        BLW = hardwareMap.dcMotor.get("BLW");
        FRW = hardwareMap.dcMotor.get("FRW");
        BRW = hardwareMap.dcMotor.get("BRW");
        LeftShoot = hardwareMap.dcMotor.get("LeftShoot");
        RightShoot = hardwareMap.dcMotor.get("RightShoot");
        Intake = hardwareMap.dcMotor.get("Intake");
        Angle1 = hardwareMap.servo.get("Angle1");
        Angle2 = hardwareMap.servo.get("Angle2");
        FlowerR = hardwareMap.servo.get("FlowerR");
        FlowerL = hardwareMap.servo.get("FlowerL");
        Stopper = hardwareMap.crservo.get("Stopper");

        // Same directions/modes as the recorder (shared method)
        TeleRecord.configureMotors(FLW, BLW, FRW, BRW, LeftShoot, RightShoot, Intake);

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
            while (index + 1 < frames.size() && frames.get(index + 1)[T] <= t) {
                index++;
            }

            double[] f = frames.get(index);

            // Drive
            FLW.setPower(clip(f[FL] * scale));
            BLW.setPower(clip(f[BL] * scale));
            FRW.setPower(clip(f[FR] * scale));
            BRW.setPower(clip(f[BR] * scale));

            // Shooter (voltage-scaled for consistent shots)
            LeftShoot.setPower(clip(f[L_SHOOT] * scale));
            RightShoot.setPower(clip(f[R_SHOOT] * scale));

            // Intake + stopper
            Intake.setPower(f[INTAKE]);
            Stopper.setPower(f[STOPPER]);

            // Servos (NaN = never commanded during recording, so leave alone)
            setServo(Angle1, f[ANGLE1]);
            setServo(Angle2, f[ANGLE2]);
            setServo(FlowerR, f[FLOWER_R]);
            setServo(FlowerL, f[FLOWER_L]);

            telemetry.addData("Playback", "%.1f / %.1f s", t, duration);
            telemetry.addData("Frame", index + "/" + frames.size());
            telemetry.addData("Voltage scale", "%.2f", scale);
            telemetry.update();
        }

        stopAll();
    }

    private void stopAll() {
        FLW.setPower(0);
        BLW.setPower(0);
        FRW.setPower(0);
        BRW.setPower(0);
        LeftShoot.setPower(0);
        RightShoot.setPower(0);
        Intake.setPower(0);
        Stopper.setPower(0);
    }

    private static void setServo(Servo s, double pos) {
        if (!Double.isNaN(pos)) s.setPosition(pos);
    }

    private boolean loadRecording() {
        try (BufferedReader r = new BufferedReader(new FileReader(FILE_PATH))) {
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
                if (p.length != COLUMNS) {
                    throw new IllegalStateException(
                            "Old/invalid file format (" + p.length + " columns). Re-record.");
                }
                double[] f = new double[COLUMNS];
                for (int i = 0; i < COLUMNS; i++) {
                    f[i] = Double.parseDouble(p[i]); // parses "NaN" correctly
                }
                frames.add(f);
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
