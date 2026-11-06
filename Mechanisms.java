package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RouteData.ANGLE_SERVO_1;
import static org.firstinspires.ftc.teamcode.RouteData.ANGLE_SERVO_2;
import static org.firstinspires.ftc.teamcode.RouteData.FLOWER_LEFT;
import static org.firstinspires.ftc.teamcode.RouteData.FLOWER_RIGHT;
import static org.firstinspires.ftc.teamcode.RouteData.INTAKE;
import static org.firstinspires.ftc.teamcode.RouteData.LEFT_SHOOTER;
import static org.firstinspires.ftc.teamcode.RouteData.MECHANISM_COUNT;
import static org.firstinspires.ftc.teamcode.RouteData.RIGHT_SHOOTER;
import static org.firstinspires.ftc.teamcode.RouteData.STOPPER;
import static org.firstinspires.ftc.teamcode.RouteData.STOPPER_PARTNER;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * The non-drive hardware: shooter, intake, stopper and servos.
 *
 * A "state" is an array with one number per output (see RouteData for the layout).
 *
 * updateStateFromGamepad() uses the SAME controls as your real TeleOp
 * (RandomBS3Conglomerate), so what you record is exactly what you normally drive with.
 * sendToHardware() applies a recorded state to the robot during playback.
 */
public class Mechanisms {
    private final DcMotor leftShooterMotor, rightShooterMotor, intakeMotor;
    private final Servo angleServo1, angleServo2, flowerServoRight, flowerServoLeft;
    private final CRServo stopperServo, stopperPartnerServo;

    /** Multiplier for shooter power (makes up for a different battery voltage). */
    private double shooterPowerMultiplier = 1.0;

    public Mechanisms(HardwareMap hardwareMap) {
        leftShooterMotor = hardwareMap.dcMotor.get("LeftShoot");
        rightShooterMotor = hardwareMap.dcMotor.get("RightShoot");
        intakeMotor = hardwareMap.dcMotor.get("Intake");
        angleServo1 = hardwareMap.servo.get("Angle1");
        angleServo2 = hardwareMap.servo.get("Angle2");
        flowerServoRight = hardwareMap.servo.get("FlowerR");
        flowerServoLeft = hardwareMap.servo.get("FlowerL");
        stopperServo = hardwareMap.crservo.get("Stopper");
        stopperPartnerServo = hardwareMap.crservo.get("ThisOne2");
    }

    public void setShooterPowerMultiplier(double multiplier) {
        shooterPowerMultiplier = multiplier;
    }

    /** A fresh state: all powers 0, all servo positions "not commanded yet" (NaN). */
    public static double[] newEmptyState() {
        double[] state = new double[MECHANISM_COUNT];
        state[ANGLE_SERVO_1] = Double.NaN;
        state[ANGLE_SERVO_2] = Double.NaN;
        state[FLOWER_RIGHT] = Double.NaN;
        state[FLOWER_LEFT] = Double.NaN;
        return state;
    }

    /**
     * Update a state from gamepad2. Motor powers are rewritten every call; servo
     * positions only change when their button is pressed (otherwise they hold).
     */
    public static void updateStateFromGamepad(Gamepad gamepad, double[] state) {
        // Shooter
        double shooterPower = gamepad.dpad_up ? 0.5 : (gamepad.dpad_down ? -0.5 : 0.0);
        state[LEFT_SHOOTER] = shooterPower;
        state[RIGHT_SHOOTER] = shooterPower;

        // Intake
        state[INTAKE] = gamepad.triangle ? -1.0 : (gamepad.square ? 1.0 : 0.0);

        // Stopper and its partner servo (they always move in opposite directions)
        if (gamepad.circle) {
            state[STOPPER] = 1.0;
            state[STOPPER_PARTNER] = -1.0;
        } else if (gamepad.cross) {
            state[STOPPER] = -1.0;
            state[STOPPER_PARTNER] = 1.0;
        } else {
            state[STOPPER] = 0.0;
            state[STOPPER_PARTNER] = 0.0;
        }

        // Angle servos
        if (gamepad.right_bumper) {
            state[ANGLE_SERVO_1] = 0.5;
            state[ANGLE_SERVO_2] = 0.0;
        } else if (gamepad.left_bumper) {
            state[ANGLE_SERVO_1] = 0.0;
            state[ANGLE_SERVO_2] = 0.5;
        }

        // Flower ramp servos
        if (gamepad.right_trigger > 0.1) {
            state[FLOWER_RIGHT] = 1.0;
            state[FLOWER_LEFT] = 0.0;
        } else if (gamepad.left_trigger > 0.1) {
            state[FLOWER_RIGHT] = 0.0;
            state[FLOWER_LEFT] = 1.0;
        }
    }

    /** Send a state to the hardware. */
    public void sendToHardware(double[] state) {
        leftShooterMotor.setPower(limitToPlusMinusOne(state[LEFT_SHOOTER] * shooterPowerMultiplier));
        rightShooterMotor.setPower(limitToPlusMinusOne(state[RIGHT_SHOOTER] * shooterPowerMultiplier));
        intakeMotor.setPower(state[INTAKE]);
        stopperServo.setPower(state[STOPPER]);
        stopperPartnerServo.setPower(state[STOPPER_PARTNER]);
        setServoIfCommanded(angleServo1, state[ANGLE_SERVO_1]);
        setServoIfCommanded(angleServo2, state[ANGLE_SERVO_2]);
        setServoIfCommanded(flowerServoRight, state[FLOWER_RIGHT]);
        setServoIfCommanded(flowerServoLeft, state[FLOWER_LEFT]);
    }

    /** Stop everything that spins (servos stay where they are). */
    public void stopAllMotion() {
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);
        intakeMotor.setPower(0);
        stopperServo.setPower(0);
        stopperPartnerServo.setPower(0);
    }

    /** Lowest battery voltage reported by any hub (12.0 if none can be read). */
    public static double readBatteryVoltage(HardwareMap hardwareMap) {
        double lowestVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) lowestVoltage = Math.min(lowestVoltage, voltage);
        }
        return lowestVoltage == Double.POSITIVE_INFINITY ? 12.0 : lowestVoltage;
    }

    /** NaN means "never commanded during recording", so the servo is left alone. */
    private static void setServoIfCommanded(Servo servo, double position) {
        if (!Double.isNaN(position)) servo.setPosition(position);
    }

    private static double limitToPlusMinusOne(double power) {
        return Math.max(-1.0, Math.min(1.0, power));
    }
}
