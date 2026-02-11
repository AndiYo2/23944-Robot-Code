package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import Constants.NamingConstants;
import Constants.ShooterConstants;
import Constants.TurretConstants;
import utility.RobotHardware;

/**
 * Shooter + Turret + Hood diagnostic TeleOp.
 *
 * CONTROLS:
 *   Left Stick Y  = flywheel power (up = forward spin)
 *   Right Bumper   = motor 1 only (hold)
 *   Left Bumper    = motor 2 only (hold)
 *   A button       = reset encoders
 *
 *   Right Stick X  = turret aiming (proportional: full right = full CW, release = center)
 *
 *   D-Pad Up       = increase hood angle (hold to ramp)
 *   D-Pad Down     = decrease hood angle (hold to ramp)
 */
@TeleOp(name = "Shooter Test", group = "Tests")
public class ShooterTest extends OpMode {

    private RobotHardware robot;
    private DcMotorEx motor1, motor2;

    private double hoodAngle = ShooterConstants.HOOD_DEFAULT_ANGLE;
    private static final double HOOD_INCREMENT = 0.5; // degrees per loop while held

    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        motor1 = robot.shooterMotor1;
        motor2 = robot.shooterMotor2;

        // Reset encoders, run without encoder for direct power control
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set hood to default
        robot.shooterHood.setPosition(hoodAngleToServo(hoodAngle));

        // Center turret
        robot.turretServo.setPosition(TurretConstants.SERVO_CENTER_POSITION);

        telemetry.addLine("=== SHOOTER TEST ===");
        telemetry.addLine("LStick Y = Flywheel | RStick X = Turret");
        telemetry.addLine("DPad U/D = Hood | RB/LB = Motor isolate");
        telemetry.addLine("A = Reset Encoders");
        telemetry.update();
    }

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        // ==================== SHOOTER FLYWHEEL ====================
        double power = -gamepad1.left_stick_y;

        boolean motor1Only = gamepad1.right_bumper;
        boolean motor2Only = gamepad1.left_bumper;

        double power1, power2;
        if (motor1Only && !motor2Only) {
            power1 = power;
            power2 = 0;
        } else if (motor2Only && !motor1Only) {
            power1 = 0;
            power2 = power;
        } else {
            power1 = power;
            power2 = power;
        }

        motor1.setPower(power1);
        motor2.setPower(power2);

        // Reset encoders on A press
        if (gamepad1.a) {
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // ==================== TURRET ====================
        // Right stick X maps proportionally to turret range
        // Full right (+1) = HARD_STOP_CW, full left (-1) = HARD_STOP_CCW, center = 0
        double turretInput = gamepad1.right_stick_x;
        double turretDegrees;
        if (turretInput >= 0) {
            turretDegrees = turretInput * TurretConstants.HARD_STOP_CW;
        } else {
            turretDegrees = -turretInput * TurretConstants.HARD_STOP_CCW;
        }

        double turretServoPos = TurretConstants.SERVO_CENTER_POSITION
                + (turretDegrees * TurretConstants.GEAR_RATIO / TurretConstants.SERVO_DEGREES_PER_UNIT);
        turretServoPos = Math.max(0.0, Math.min(1.0, turretServoPos));
        robot.turretServo.setPosition(turretServoPos);

        // ==================== HOOD ====================
        if (gamepad1.dpad_up) {
            hoodAngle += HOOD_INCREMENT;
        }
        if (gamepad1.dpad_down) {
            hoodAngle -= HOOD_INCREMENT;
        }
        hoodAngle = Math.max(ShooterConstants.HOOD_MIN_ANGLE,
                Math.min(ShooterConstants.HOOD_MAX_ANGLE, hoodAngle));

        double hoodServoPos = hoodAngleToServo(hoodAngle);
        robot.shooterHood.setPosition(hoodServoPos);

        // ==================== DIAGNOSTICS ====================
        double vel1 = motor1.getVelocity();
        double vel2 = motor2.getVelocity();
        int pos1 = motor1.getCurrentPosition();
        int pos2 = motor2.getCurrentPosition();
        double current1 = motor1.getCurrent(CurrentUnit.AMPS);
        double current2 = motor2.getCurrent(CurrentUnit.AMPS);

        telemetry.addData("Loop", "%.1f ms (%.0f Hz)", loopMs, loopMs > 0 ? 1000.0 / loopMs : 0);
        telemetry.addLine();

        // Shooter section
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("Commanded Power", "%.2f", power);
        telemetry.addLine("-- Motor 1 (C0, Left) --");
        telemetry.addData("  Power", "%.2f", power1);
        telemetry.addData("  Velocity", "%.1f ticks/s", vel1);
        telemetry.addData("  Position", "%d ticks", pos1);
        telemetry.addData("  Current", "%.2f A", current1);
        telemetry.addLine("-- Motor 2 (C1, Right) --");
        telemetry.addData("  Power", "%.2f", power2);
        telemetry.addData("  Velocity", "%.1f ticks/s", vel2);
        telemetry.addData("  Position", "%d ticks", pos2);
        telemetry.addData("  Current", "%.2f A", current2);
        telemetry.addLine("-- Comparison --");
        telemetry.addData("  Vel Delta", "%.1f ticks/s", vel1 - vel2);
        telemetry.addData("  Pos Delta", "%d ticks", pos1 - pos2);
        telemetry.addData("  Current Delta", "%.2f A", current1 - current2);

        if (Math.abs(power) > 0.1) {
            if (Math.abs(vel1) < 10 && Math.abs(vel2) > 100) {
                telemetry.addLine(">>> MOTOR 1 NOT SPINNING <<<");
            } else if (Math.abs(vel2) < 10 && Math.abs(vel1) > 100) {
                telemetry.addLine(">>> MOTOR 2 NOT SPINNING <<<");
            } else if (Math.abs(vel1 - vel2) > 200) {
                telemetry.addLine(">>> LARGE VELOCITY MISMATCH <<<");
            }
        }

        telemetry.addLine();

        // Turret section
        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Stick Input", "%.2f", turretInput);
        telemetry.addData("Target Degrees", "%.1f deg", turretDegrees);
        telemetry.addData("Servo Position", "%.4f", turretServoPos);
        telemetry.addData("Hard Stops", "CCW %.0f / CW %.0f",
                TurretConstants.HARD_STOP_CCW, TurretConstants.HARD_STOP_CW);
        telemetry.addLine();

        // Hood section
        telemetry.addLine("=== HOOD ===");
        telemetry.addData("Hood Angle", "%.1f deg", hoodAngle);
        telemetry.addData("Servo Position", "%.4f", hoodServoPos);
        telemetry.addData("Range", "%.0f - %.0f deg",
                ShooterConstants.HOOD_MIN_ANGLE, ShooterConstants.HOOD_MAX_ANGLE);

        telemetry.update();
    }

    @Override
    public void stop() {
        motor1.setPower(0);
        motor2.setPower(0);
        robot.turretServo.setPosition(TurretConstants.SERVO_CENTER_POSITION);
    }

    /**
     * Convert hood angle to servo position.
     * Mirrors Shooter subsystem logic: 30deg -> 1.0, 63deg -> 0.34
     */
    private double hoodAngleToServo(double angleDegrees) {
        double angleRange = ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE;
        double servoRange = ShooterConstants.HOOD_SERVO_AT_MIN_ANGLE - ShooterConstants.HOOD_SERVO_AT_MAX_ANGLE;
        double normalized = (angleDegrees - ShooterConstants.HOOD_MIN_ANGLE) / angleRange;
        double position = ShooterConstants.HOOD_SERVO_AT_MIN_ANGLE - (normalized * servoRange);
        return Math.max(ShooterConstants.HOOD_SERVO_AT_MAX_ANGLE,
                Math.min(ShooterConstants.HOOD_SERVO_AT_MIN_ANGLE, position));
    }
}