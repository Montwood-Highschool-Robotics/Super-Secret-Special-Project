package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TeleRecord;

import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.util.ArrayList;

@Autonomous
public class  Playback extends LinearOpMode {
    private DcMotor FLW;
    private DcMotor BLW;
    private DcMotor FRW;
    private DcMotor BRW;


    private ArrayList<TeleRecord.GamepadSnapshot> playlist;
    private final String FILE_PATH = "/sdcard/FIRST/recorded_input.dat";

    @Override
    public void runOpMode() {
        // Initialize your hardware maps here (Exactly identical to your recorder layout)


        FLW = hardwareMap.dcMotor.get("FLW");
        BLW = hardwareMap.dcMotor.get("BLW");
        BRW = hardwareMap.dcMotor.get("BRW");
        FRW = hardwareMap.dcMotor.get("FRW");




        telemetry.addData("Status", "Loading recording file...");
        telemetry.update();

        try (ObjectInputStream ois = new ObjectInputStream(new FileInputStream(FILE_PATH))) {
            playlist = (ArrayList<TeleRecord.GamepadSnapshot>) ois.readObject();
            telemetry.addData("Status", "Loaded successfully! Frames: " + playlist.size());
        } catch (Exception e) {
            telemetry.addData("Error", "Could not load file: " + e.getMessage());
        }
        telemetry.update();



        waitForStart();

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

        long playbackStartTime = System.currentTimeMillis();
        int frameIndex = 0;

        while (opModeIsActive() && frameIndex < playlist.size()) {
            long elapsed = System.currentTimeMillis() - playbackStartTime;


            TeleRecord.GamepadSnapshot currentFrame = playlist.get(frameIndex);
            while (frameIndex < playlist.size() && playlist.get(frameIndex).timestamp <= elapsed) {
                currentFrame = playlist.get(frameIndex);
                frameIndex++;
            }



            telemetry.addData("Playback Time (s)", elapsed / 1000.0);
            telemetry.addData("Current Frame", frameIndex + "/" + playlist.size());
            telemetry.update();
        }
    }
}
