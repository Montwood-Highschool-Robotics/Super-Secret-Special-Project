
package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.FileOutputStream;
import java.io.ObjectOutputStream;
import java.io.Serializable;
import java.util.ArrayList;

@TeleOp
public class TeleOpRecorderRevival extends LinearOpMode {
    
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
    // Path to save on the Control Hub internal storage enter when you know what it is
    private final String FILE_PATH = "/sdcard/FIRST/recorded_input.dat";

    @Override
    public void runOpMode() {
        // Initialize your hardware maps here (ex. drive motors)

        telemetry.addData("Status", "Ready. Press Play to start recording.");
        telemetry.update();

        waitForStart();

        long startTime = System.currentTimeMillis();

        while (opModeIsActive()) {
            long elapsed = System.currentTimeMillis() - startTime;


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
