package teleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import Constants.NamingConstants;

/**
 * Diagnostic TeleOp for testing flywheel motors individually.
 *
 * Controls:
 *   Left stick Y  = power to BOTH motors (up = forward)
 *   Right bumper   = run motor 1 only (hold)
 *   Left bumper    = run motor 2 only (hold)
 *   (no bumper)    = run both motors
 *   A button       = reset encoders
 *
 * Telemetry shows per-motor: velocity, position, current, power, and delta between the two.
 */
@TeleOp(name = "Shooter Test", group = "Test")
public class ShooterTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Init motors directly — no subsystems needed
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter1);
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter2);

        motor2.setDirection(DcMotorEx.Direction.REVERSE);

        // Reset encoders, then run without encoder so we control power directly but can still read velocity
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        ElapsedTime loopTimer = new ElapsedTime();

        telemetry.addLine("Shooter Test Ready");
        telemetry.addLine("Left stick Y = power");
        telemetry.addLine("RB = motor1 only | LB = motor2 only");
        telemetry.addLine("A = reset encoders");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double loopMs = loopTimer.milliseconds();
            loopTimer.reset();

            // Power from joystick (up = positive = forward spin)
            double power = -gamepad1.left_stick_y;

            // Bumpers isolate individual motors for testing
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

            // Read all diagnostics
            double vel1 = motor1.getVelocity();
            double vel2 = motor2.getVelocity();
            int pos1 = motor1.getCurrentPosition();
            int pos2 = motor2.getCurrentPosition();
            double current1 = motor1.getCurrent(CurrentUnit.AMPS);
            double current2 = motor2.getCurrent(CurrentUnit.AMPS);

            double velDelta = vel1 - vel2;

            // Display
            telemetry.addData("Loop", "%.1f ms (%.0f Hz)", loopMs, loopMs > 0 ? 1000.0 / loopMs : 0);
            telemetry.addData("Commanded Power", "%.2f", power);
            telemetry.addLine();

            telemetry.addLine("--- MOTOR 1 (C0, Left) ---");
            telemetry.addData("  Power", "%.2f", power1);
            telemetry.addData("  Velocity", "%.1f ticks/s", vel1);
            telemetry.addData("  Position", "%d ticks", pos1);
            telemetry.addData("  Current", "%.2f A", current1);
            telemetry.addLine();

            telemetry.addLine("--- MOTOR 2 (C1, Right) ---");
            telemetry.addData("  Power", "%.2f", power2);
            telemetry.addData("  Velocity", "%.1f ticks/s", vel2);
            telemetry.addData("  Position", "%d ticks", pos2);
            telemetry.addData("  Current", "%.2f A", current2);
            telemetry.addLine();

            telemetry.addLine("--- COMPARISON ---");
            telemetry.addData("  Vel Delta (M1-M2)", "%.1f ticks/s", velDelta);
            telemetry.addData("  Pos Delta (M1-M2)", "%d ticks", pos1 - pos2);
            telemetry.addData("  Current Delta", "%.2f A", current1 - current2);

            // Flag obvious problems
            if (Math.abs(power) > 0.1) {
                if (Math.abs(vel1) < 10 && Math.abs(vel2) > 100) {
                    telemetry.addLine(">>> MOTOR 1 NOT SPINNING <<<");
                } else if (Math.abs(vel2) < 10 && Math.abs(vel1) > 100) {
                    telemetry.addLine(">>> MOTOR 2 NOT SPINNING <<<");
                } else if (Math.abs(velDelta) > 200) {
                    telemetry.addLine(">>> LARGE VELOCITY MISMATCH <<<");
                }
            }

            telemetry.update();
        }

        motor1.setPower(0);
        motor2.setPower(0);
    }
}
