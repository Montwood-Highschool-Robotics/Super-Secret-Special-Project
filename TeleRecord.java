package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import java.io.FileOutputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.ArrayList;

@TeleOp
public class TeleRecord extends LinearOpMode {

    private DcMotor FLW;
    private DcMotor BLW;
    private DcMotor FRW;
    private DcMotor BRW;

    // Denominator is a correction factor to keep power scale between -1 and 1





    // A lightweight inner class to hold static snapshots of the FTC Gamepad

    public static class GamepadSnapshot implements Serializable {
        private static final long serialVersionUID = 1L;

        public long timestamp;
        public float left_stick_x, left_stick_y, right_stick_x, right_stick_y;
        public float left_trigger, right_trigger;
        public boolean a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right;
        public boolean left_bumper, right_bumper, left_stick_button, right_stick_button;

        public GamepadSnapshot(long timestamp, com.qualcomm.robotcore.hardware.Gamepad g) {
            this.timestamp = timestamp;
            this.left_stick_x = g.left_stick_x;
            this.left_stick_y = g.left_stick_y;
            this.right_stick_x = g.right_stick_x;
            this.right_stick_y = g.right_stick_y;
            this.left_trigger = g.left_trigger;
            this.right_trigger = g.right_trigger;
            this.a = g.a;
            this.b = g.b;
            this.x = g.x;
            this.y = g.y;
            this.dpad_up = g.dpad_up;
            this.dpad_down = g.dpad_down;
            this.dpad_left = g.dpad_left;
            this.dpad_right = g.dpad_right;
            this.left_bumper = g.left_bumper;
            this.right_bumper = g.right_bumper;
            this.left_stick_button = g.left_stick_button;
            this.right_stick_button = g.right_stick_button;
        }
    }

    private ArrayList<GamepadSnapshot> recordedFrames = new ArrayList<>();
    // Path to save on the Control Hub internal storage
    private final String FILE_PATH = "/sdcard/FIRST/recorded_input.dat";

    @Override
    public void runOpMode() {
        // Initialize your hardware maps here (e.g., drive motors)
        FLW = hardwareMap.dcMotor.get("FLW");
        BLW = hardwareMap.dcMotor.get("BLW");
        BRW = hardwareMap.dcMotor.get("BRW");
        FRW = hardwareMap.dcMotor.get("FRW");






        telemetry.addData("Status", "Ready. Press Play to start recording.");
        telemetry.update();

        waitForStart();
        long startTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            long elapsed = System.currentTimeMillis() - startTime;



            // Denominator is a correction factor to keep power scale between -1 and 1
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x *.8;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            FLW.setPower(frontLeftPower);
            BLW.setPower(backLeftPower);
            FRW.setPower(frontRightPower);
            BRW.setPower(backRightPower);

            GamepadSnapshot snapshot = new GamepadSnapshot(elapsed, gamepad1);
            recordedFrames.add(snapshot);


            if (gamepad1.start) {
                telemetry.addData("Status", "Saving recording...");
                telemetry.update();
                saveRecording();
                break;
            }

            telemetry.addData("Recording Time (s)", elapsed / 1000.0);
            telemetry.addData("Frames Recorded", recordedFrames.size());
            telemetry.update();

            sleep(10);
        }
    }

    private void saveRecording() {
        try (ObjectOutputStream oos = new ObjectOutputStream(new FileOutputStream(FILE_PATH))) {
            oos.writeObject(recordedFrames);
            telemetry.addData("Success", "Saved to " + FILE_PATH);
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to save: " + e.getMessage());
        }
        telemetry.update();
        sleep(2000);
    }
}
