package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RouteData.ANGLE1;
import static org.firstinspires.ftc.teamcode.RouteData.ANGLE2;
import static org.firstinspires.ftc.teamcode.RouteData.FLOWER_L;
import static org.firstinspires.ftc.teamcode.RouteData.FLOWER_R;
import static org.firstinspires.ftc.teamcode.RouteData.INTAKE;
import static org.firstinspires.ftc.teamcode.RouteData.L_SHOOT;
import static org.firstinspires.ftc.teamcode.RouteData.MECH_COUNT;
import static org.firstinspires.ftc.teamcode.RouteData.R_SHOOT;
import static org.firstinspires.ftc.teamcode.RouteData.STOPPER;
import static org.firstinspires.ftc.teamcode.RouteData.THIS_ONE_2;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * The non-drive hardware (shooter, intake, stopper, servos).
 *
 * readGamepad() uses the SAME controls as your real TeleOp (RandomBS3Conglomerate),
 * so what you record is exactly what you normally drive with.
 * apply() sends a recorded state to the hardware during playback.
 */
public class Mechanisms {
    private final DcMotor leftShoot, rightShoot, intake;
    private final Servo angle1, angle2, flowerR, flowerL;
    private final CRServo stopper, thisOne2;

    /** Multiplier for shooter power (compensates for a different battery voltage). */
    private double shooterScale = 1.0;

    public Mechanisms(HardwareMap hw) {
        leftShoot = hw.dcMotor.get("LeftShoot");
        rightShoot = hw.dcMotor.get("RightShoot");
        intake = hw.dcMotor.get("Intake");
        angle1 = hw.servo.get("Angle1");
        angle2 = hw.servo.get("Angle2");
        flowerR = hw.servo.get("FlowerR");
        flowerL = hw.servo.get("FlowerL");
        stopper = hw.crservo.get("Stopper");
        thisOne2 = hw.crservo.get("ThisOne2");
    }

    public void setShooterScale(double scale) {
        shooterScale = scale;
    }

    /** A fresh state: all powers 0, all servo positions "not commanded yet" (NaN). */
    public static double[] newState() {
        double[] s = new double[MECH_COUNT];
        s[ANGLE1] = Double.NaN;
        s[ANGLE2] = Double.NaN;
        s[FLOWER_R] = Double.NaN;
        s[FLOWER_L] = Double.NaN;
        return s;
    }

    /**
     * Update a state from gamepad2. Motor powers are rewritten every call; servo
     * positions only change when their button is pressed (they hold otherwise).
     */
    public static void readGamepad(Gamepad g, double[] s) {
        // Shooter
        double shoot = g.dpad_up ? 0.5 : (g.dpad_down ? -0.5 : 0.0);
        s[L_SHOOT] = shoot;
        s[R_SHOOT] = shoot;

        // Intake
        s[INTAKE] = g.triangle ? -1.0 : (g.square ? 1.0 : 0.0);

        // Stopper + its partner servo
        if (g.circle) {
            s[STOPPER] = 1.0;
            s[THIS_ONE_2] = -1.0;
        } else if (g.cross) {
            s[STOPPER] = -1.0;
            s[THIS_ONE_2] = 1.0;
        } else {
            s[STOPPER] = 0.0;
            s[THIS_ONE_2] = 0.0;
        }

        // Angle
        if (g.right_bumper) {
            s[ANGLE1] = 0.5;
            s[ANGLE2] = 0.0;
        } else if (g.left_bumper) {
            s[ANGLE1] = 0.0;
            s[ANGLE2] = 0.5;
        }

        // Flower ramp
        if (g.right_trigger > 0.1) {
            s[FLOWER_R] = 1.0;
            s[FLOWER_L] = 0.0;
        } else if (g.left_trigger > 0.1) {
            s[FLOWER_R] = 0.0;
            s[FLOWER_L] = 1.0;
        }
    }

    /** Send a state to the hardware. */
    public void apply(double[] s) {
        leftShoot.setPower(clip(s[L_SHOOT] * shooterScale));
        rightShoot.setPower(clip(s[R_SHOOT] * shooterScale));
        intake.setPower(s[INTAKE]);
        stopper.setPower(s[STOPPER]);
        thisOne2.setPower(s[THIS_ONE_2]);
        setServo(angle1, s[ANGLE1]);
        setServo(angle2, s[ANGLE2]);
        setServo(flowerR, s[FLOWER_R]);
        setServo(flowerL, s[FLOWER_L]);
    }

    /** Stop everything that spins (servos stay where they are). */
    public void stop() {
        leftShoot.setPower(0);
        rightShoot.setPower(0);
        intake.setPower(0);
        stopper.setPower(0);
        thisOne2.setPower(0);
    }

    /** Lowest battery voltage reported by any hub (12.0 if none is readable). */
    public static double batteryVoltage(HardwareMap hw) {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hw.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > 0) result = Math.min(result, v);
        }
        return result == Double.POSITIVE_INFINITY ? 12.0 : result;
    }

    private static void setServo(Servo s, double pos) {
        if (!Double.isNaN(pos)) s.setPosition(pos);
    }

    private static double clip(double p) {
        return Math.max(-1.0, Math.min(1.0, p));
    }
}
