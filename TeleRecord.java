package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.util.ArrayList;
import java.util.Locale;

/**
 * Drive normally (gamepad1 = drive, gamepad2 = mechanisms) for up to 30 seconds.
 * Every loop the wheel powers AND mechanism outputs are logged with a timestamp.
 * When the 30 s are up (or you press START on gamepad1) the log is saved to a
 * CSV file that the Playback OpMode replays.
 */
@TeleOp(name = "Record Drive (30s)", group = "Record")
public class TeleRecord extends LinearOpMode {

    public static final String FILE_PATH = "/sdcard/FIRST/recorded_drive.csv";
    public static final double RECORD_SECONDS = 30.0;

    // CSV column layout (shared with Playback)
    public static final int T = 0;
    public static final int FL = 1, BL = 2, FR = 3, BR = 4;
    public static final int L_SHOOT = 5, R_SHOOT = 6;
    public static final int INTAKE = 7, STOPPER = 8;
    public static final int ANGLE1 = 9, ANGLE2 = 10;
    public static final int FLOWER_R = 11, FLOWER_L = 12;
    public static final int COLUMNS = 13;

    private DcMotor FLW, BLW, FRW, BRW, LeftShoot, RightShoot, Intake;
    private Servo Angle1, Angle2, FlowerR, FlowerL;
    private CRServo Stopper;

    private final ArrayList<double[]> frames = new ArrayList<>();

    /**
     * Single source of truth for motor modes/directions. Playback calls this
     * too, so the two OpModes can never get out of sync.
     */
    public static void configureMotors(DcMotor FLW, DcMotor BLW, DcMotor FRW, DcMotor BRW,
                                       DcMotor LeftShoot, DcMotor RightShoot, DcMotor Intake) {
        for (DcMotor m : new DcMotor[]{FLW, BLW, FRW, BRW, LeftShoot, RightShoot, Intake}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        FRW.setDirection(DcMotorSimple.Direction.REVERSE);
        BRW.setDirection(DcMotorSimple.Direction.REVERSE);
    }

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

        configureMotors(FLW, BLW, FRW, BRW, LeftShoot, RightShoot, Intake);

        telemetry.addData("Status", "Ready. Press PLAY - recording starts immediately.");
        telemetry.addData("Length", "%.0f seconds (START on gamepad1 ends early)", RECORD_SECONDS);
        telemetry.update();

        waitForStart();

        double batteryVoltage = getBatteryVoltage();
        ElapsedTime timer = new ElapsedTime();

        // Servos hold their last commanded position. NaN = "not commanded yet",
        // which Playback treats as "leave this servo alone".
        double angle1Pos = Double.NaN, angle2Pos = Double.NaN;
        double flowerRPos = Double.NaN, flowerLPos = Double.NaN;

        while (opModeIsActive() && timer.seconds() < RECORD_SECONDS) {
            double t = timer.seconds();

            // ---- Drive (gamepad1) ----
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

            // ---- Shooter ----
            double shootPower;
            if (gamepad2.dpad_up) {
                shootPower = 0.5;
            } else if (gamepad2.dpad_down) {
                shootPower = -0.5;
            } else {
                shootPower = 0;
            }
            LeftShoot.setPower(shootPower);
            RightShoot.setPower(shootPower);

            // ---- Intake ----
            double intakePower;
            if (gamepad2.triangle) {
                intakePower = -1;
            } else if (gamepad2.square) {
                intakePower = 1;
            } else {
                intakePower = 0;
            }
            Intake.setPower(intakePower);

            // ---- Angle ----
            if (gamepad2.right_bumper) {
                angle1Pos = 0.5;
                angle2Pos = 0;
                Angle1.setPosition(angle1Pos);
                Angle2.setPosition(angle2Pos);
            } else if (gamepad2.left_bumper) {
                angle1Pos = 0;
                angle2Pos = 0.5;
                Angle1.setPosition(angle1Pos);
                Angle2.setPosition(angle2Pos);
            }

            // ---- Flower ramp ----
            if (gamepad2.right_trigger > 0.1) {
                flowerRPos = 1;
                flowerLPos = 0;
                FlowerR.setPosition(flowerRPos);
                FlowerL.setPosition(flowerLPos);
            } else if (gamepad2.left_trigger > 0.1) {
                flowerRPos = 0;
                flowerLPos = 1;
                FlowerR.setPosition(flowerRPos);
                FlowerL.setPosition(flowerLPos);
            }

            // ---- Stopper (continuous servo) ----
            double stopperPower;
            if (gamepad2.square) {
                stopperPower = 1;
            } else if (gamepad2.cross) {
                stopperPower = -1;
            } else {
                stopperPower = 0;
            }
            Stopper.setPower(stopperPower);

            // ---- Log everything ----
            double[] frame = new double[COLUMNS];
            frame[T] = t;
            frame[FL] = fl;
            frame[BL] = bl;
            frame[FR] = fr;
            frame[BR] = br;
            frame[L_SHOOT] = shootPower;
            frame[R_SHOOT] = shootPower;
            frame[INTAKE] = intakePower;
            frame[STOPPER] = stopperPower;
            frame[ANGLE1] = angle1Pos;
            frame[ANGLE2] = angle2Pos;
            frame[FLOWER_R] = flowerRPos;
            frame[FLOWER_L] = flowerLPos;
            frames.add(frame);

            if (gamepad1.start) break;

            telemetry.addData("Recording", "%.1f / %.0f s", t, RECORD_SECONDS);
            telemetry.addData("Frames", frames.size());
            telemetry.update();

            sleep(10);
        }

        stopAll();
        saveRecording(batteryVoltage, frames.isEmpty() ? 0 : frames.get(frames.size() - 1)[T]);
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
                    StringBuilder sb = new StringBuilder();
                    for (int i = 0; i < f.length; i++) {
                        if (i > 0) sb.append(',');
                        sb.append(String.format(Locale.US, "%.4f", f[i])); // NaN -> "NaN"
                    }
                    w.write(sb.toString());
                    w.newLine();
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
