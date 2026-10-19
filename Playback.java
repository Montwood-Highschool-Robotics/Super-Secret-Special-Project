package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeleOps.TeleOpRecorderRevival;

import java.io.FileInputStream;
import java.io.ObjectInputStream;
import java.util.ArrayList;

@Autonomous
public class  Playback extends LinearOpMode {

    private ArrayList<TeleOpRecorderRevival.GamepadSnapshot> playlist;
    private final String FILE_PATH = "/sdcard/FIRST/recorded_input.dat";

    @Override
    public void runOpMode() {
        // Initialize your hardware maps here (Exactly identical to your recorder layout)
        // SampleHardware robot = new SampleHardware(hardwareMap);

        telemetry.addData("Status", "Loading recording file...");
        telemetry.update();

        try (ObjectInputStream ois = new ObjectInputStream(new FileInputStream(FILE_PATH))) {
            playlist = (ArrayList<TeleOpRecorderRevival.GamepadSnapshot>) ois.readObject();
            telemetry.addData("Status", "Loaded successfully! Frames: " + playlist.size());
        } catch (Exception e) {
            telemetry.addData("Error", "Could not load file: " + e.getMessage());
        }
        telemetry.update();

        waitForStart();

        long playbackStartTime = System.currentTimeMillis();
        int frameIndex = 0;

        while (opModeIsActive() && frameIndex < playlist.size()) {
            long elapsed = System.currentTimeMillis() - playbackStartTime;


            TeleOpRecorderRevival.GamepadSnapshot currentFrame = playlist.get(frameIndex);
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
