package tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Standalone motor + encoder test TeleOp for robot rebuild hardware validation.
 *
 * Controls:
 *   Left stick Y   — set motor power
 *   Right stick Y  — set servo position (stick up = 1.0, stick down = 0.0)
 *   X              — reset encoder count
 *
 * Encoder always reads. Hardware config: motors named "motor" and "motor2", servo named "servo".
 * motor2 runs inverse to motor (opposite direction, same power).
 */
@TeleOp(name = "Motor Encoder Test", group = "Test")
public class MotorEncoderTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        Servo servo = hardwareMap.get(Servo.class, "servo");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotor.Direction.FORWARD);

        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setDirection(DcMotor.Direction.REVERSE);

        servo.setPosition(0.5);

        boolean prevX = false;

        ElapsedTime loopTimer = new ElapsedTime();
        int prevPosition = 0;
        double prevTime = 0;
        double velocityTicksPerSec = 0;

        telemetry.addLine("Motor Encoder Test Ready");
        telemetry.addLine("Left stick Y = motor | Right stick Y = servo");
        telemetry.addLine("X = reset encoder");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            // --- Reset encoder on X press ---
            boolean curX = gamepad1.x;
            if (curX && !prevX) {
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            prevX = curX;

            // --- Motor power from left stick (motor2 runs inverse) ---
            double power = -gamepad1.left_stick_y; // stick up = positive
            motor.setPower(power);
            motor2.setPower(power);

            // --- Servo position from right stick ---
            // Stick up (-1) = 1.0, center (0) = 0.5, stick down (+1) = 0.0
            double servoPos = (-gamepad1.right_stick_y + 1.0) / 2.0;
            servo.setPosition(servoPos);

            // --- Velocity calculation ---
            int currentPosition = motor.getCurrentPosition();
            double currentTime = loopTimer.seconds();
            double dt = currentTime - prevTime;
            if (dt > 0) {
                velocityTicksPerSec = (currentPosition - prevPosition) / dt;
            }
            prevPosition = currentPosition;
            prevTime = currentTime;

            // --- Telemetry ---
            telemetry.addLine("=== MOTOR 1 ===");
            telemetry.addData("Power", "%.2f", power);
            telemetry.addData("Direction", motor.getDirection());
            telemetry.addData("Position (ticks)", currentPosition);
            telemetry.addData("Velocity (ticks/s)", "%.1f", velocityTicksPerSec);
            telemetry.addData("Velocity (SDK)", "%.1f", motor.getVelocity());

            telemetry.addLine("=== MOTOR 2 (inverse) ===");
            telemetry.addData("Power ", "%.2f", power);
            telemetry.addData("Direction ", motor2.getDirection());
            telemetry.addData("Position (ticks) ", motor2.getCurrentPosition());
            telemetry.addData("Velocity (SDK) ", "%.1f", motor2.getVelocity());

            telemetry.addLine("=== SERVO ===");
            telemetry.addData("Position", "%.3f", servoPos);

            telemetry.addLine("=== LOOP ===");
            telemetry.addData("Loop dt", "%.1f ms", dt * 1000);

            telemetry.addData("\nX", "Reset encoder");
            telemetry.update();
        }

        motor.setPower(0);
        motor2.setPower(0);
    }
}