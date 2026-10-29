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
 * Drive normally for up to 30 seconds. Every loop the wheel powers are logged
 * with a timestamp. When the 30 s are up (or you press START on gamepad1)
 * the log is saved to a CSV file that the Playback OpMode replays.
 */
@TeleOp(name = "Record Drive (30s)", group = "Record")
public class TeleRecord extends LinearOpMode {

    public static final String FILE_PATH = "/sdcard/FIRST/recorded_drive.csv";
    public static final double RECORD_SECONDS = 30.0;

    private DcMotor FLW ,BLW, FRW, BRW,LeftShoot,RightShoot,Intake;
    private Servo Angle1,Angle2,FlowerR,FlowerL;
    private CRServo Stopper;

    // One row per loop: time (s) and the four wheel powers
    private final ArrayList<double[]> frames = new ArrayList<>();

    @Override
    public void runOpMode() {
        FLW = hardwareMap.dcMotor.get("FLW");
        BLW = hardwareMap.dcMotor.get("BLW");
        FRW = hardwareMap.dcMotor.get("FRW");
        BRW = hardwareMap.dcMotor.get("BRW");
        LeftShoot = hardwareMap.dcMotor.get("LeftShoot");
        RightShoot = hardwareMap.dcMotor.get("RightShoot");
        Intake = hardwareMap.dcMotor.get("Intake");
        Servo Angle1 = hardwareMap.servo.get("Angle1");
        Servo Angle2 = hardwareMap.servo.get("Angle2");
        Servo FlowerR = hardwareMap.servo.get("FlowerR");
        Servo FlowerL = hardwareMap.servo.get("FlowerL");
        CRServo Stopper = hardwareMap.crservo.get("Stopper");




        // If your robot needs reversed motors, set them here AND in Playback
        // (they must match), e.g. FLW.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{FLW, BLW, FRW, BRW,LeftShoot,RightShoot,Intake}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            BRW.setDirection(DcMotorSimple.Direction.REVERSE);
            // BRW.setDirection(DcMotorSimple.Direction.REVERSE);
            FRW.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        for (Servo s : new Servo[]{Angle1,Angle2,FlowerR,FlowerL}){

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
            if (gamepad2.dpad_up) {
                LeftShoot.setPower(0.5);
                RightShoot.setPower(0.5);

            }else if (gamepad2.dpad_down){
                LeftShoot.setPower(-.5);
                RightShoot.setPower(-0.5);
            }else {

                LeftShoot.setPower(0);
                RightShoot.setPower(0);
            }


            //Intake
            if (gamepad2.triangle){
                Intake.setPower(-1);
            }else if (gamepad2.square){
                Intake.setPower(1);
            }else {
                Intake.setPower(0);
            }


            //Angle
            if (gamepad2.right_bumper){
                Angle1.setPosition(.5);
                Angle2.setPosition(0);
            } else if (gamepad2.left_bumper) {
                Angle1.setPosition(0);
                Angle2.setPosition(0.5);
            }

            //Flower Ramp Mech

            if (gamepad2.right_trigger > 0.1){
                FlowerR.setPosition(1);
                FlowerL.setPosition(0);
            } else if (gamepad2.left_trigger > 0.1) {
                FlowerR.setPosition(0);
                FlowerL.setPosition(1);
            }

            if (gamepad2.square){
                Stopper.setPower(1);
            } else if (gamepad2.cross) {
                Stopper.setPower(-1);
            } else{
                Stopper.setPower(0);
            }


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
