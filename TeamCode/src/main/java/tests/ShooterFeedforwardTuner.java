package tests;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.NamingConstants;
import Constants.ShooterFeedforwardConstants;

/**
 * Shooter Feedforward + PID Tuning OpMode
 *
 * Uses FTCLib's SimpleMotorFeedforward combined with PIDController
 * for precise flywheel velocity control.
 *
 * CONTROLS:
 * - A: Toggle flywheel ON/OFF
 * - B: Switch tuning mode (kS → kV → kI → kP)
 * - DPAD UP/DOWN: Adjust current parameter
 * - X: Reset to defaults
 * - Y: Run kS finder (slowly ramps power)
 * - RIGHT BUMPER: Simulate ball shot (disturbance test)
 *
 * TUNING PROCESS:
 * 1. Press Y to find kS (note the power when flywheel starts moving)
 * 2. Set kS, then run at full power to find max velocity for kV
 * 3. Enable flywheel (A), tune kI until steady-state error is minimal
 * 4. Tune kP for faster recovery (use RB to simulate disturbance)
 */
@TeleOp(name = "Shooter Feedforward Tuner", group = "Tests")
public class ShooterFeedforwardTuner extends OpMode {

    private DcMotorEx flywheelMotor1, flywheelMotor2;
    private SimpleMotorFeedforward feedforward;
    private PIDController pidController;

    private ElapsedTime loopTimer = new ElapsedTime();
    private ElapsedTime buttonDebounce = new ElapsedTime();
    private ElapsedTime kSRampTimer = new ElapsedTime();

    // State
    private boolean flywheelEnabled = false;
    private boolean kSFinderActive = false;
    private double kSRampPower = 0.0;

    // Tuning mode
    private enum TuningMode { kS, kV, kI, kP }
    private TuningMode currentMode = TuningMode.kS;

    // Statistics
    private double maxVelocityObserved = 0;
    private double minError = Double.MAX_VALUE;
    private double maxError = 0;
    private double errorSum = 0;
    private int errorCount = 0;

    // Integral accumulator (for manual tracking)
    private double integralSum = 0;

    @Override
    public void init() {
        // Initialize motors in RUN_WITHOUT_ENCODER for external control
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter1);
        flywheelMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter2);
        flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize FTCLib controllers
        updateControllers();

        telemetry.addLine("=== SHOOTER FEEDFORWARD TUNER ===");
        telemetry.addLine("A: Toggle flywheel | B: Switch mode");
        telemetry.addLine("DPAD: Adjust values | Y: Find kS");
        telemetry.addLine("RB: Simulate shot disturbance");
        telemetry.update();
    }

    private void updateControllers() {
        feedforward = new SimpleMotorFeedforward(
            ShooterFeedforwardConstants.kS,
            ShooterFeedforwardConstants.kV,
            ShooterFeedforwardConstants.kA
        );

        pidController = new PIDController(
            ShooterFeedforwardConstants.kP,
            ShooterFeedforwardConstants.kI,
            ShooterFeedforwardConstants.kD
        );

        // Reset integral
        pidController.reset();
        integralSum = 0;
    }

    @Override
    public void loop() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        handleInput();

        double currentVelocity1 = flywheelMotor1.getVelocity();
        double currentVelocity2 = flywheelMotor2.getVelocity();
        double avgVelocity = (currentVelocity1 + currentVelocity2) / 2.0;

        // Track max velocity
        if (avgVelocity > maxVelocityObserved) {
            maxVelocityObserved = avgVelocity;
        }

        double power = 0;
        double ffOutput = 0;
        double pidOutput = 0;
        double targetVelocity = ShooterFeedforwardConstants.TUNING_VELOCITY;

        if (kSFinderActive) {
            // kS Finder: slowly ramp power until flywheel moves
            power = kSRampPower;
            if (kSRampTimer.seconds() > 0.1) {
                kSRampPower += 0.005;
                kSRampTimer.reset();
            }

            // Stop when flywheel starts moving
            if (avgVelocity > 10) {
                kSFinderActive = false;
                ShooterFeedforwardConstants.kS = kSRampPower - 0.01; // Slightly below movement threshold
            }

            if (kSRampPower > 0.5) {
                kSFinderActive = false; // Safety cutoff
            }

        } else if (flywheelEnabled) {
            // Feedforward + PID control
            ffOutput = feedforward.calculate(targetVelocity);

            double error = targetVelocity - avgVelocity;

            // Manual integral with anti-windup
            integralSum += error * dt;
            integralSum = Math.max(-ShooterFeedforwardConstants.INTEGRAL_MAX / ShooterFeedforwardConstants.kI,
                         Math.min(ShooterFeedforwardConstants.INTEGRAL_MAX / ShooterFeedforwardConstants.kI, integralSum));

            pidOutput = pidController.calculate(avgVelocity, targetVelocity);

            power = ffOutput + pidOutput;
            power = Math.max(0, Math.min(1.0, power)); // Clamp 0-1

            // Track error statistics
            double absError = Math.abs(error);
            if (absError < minError) minError = absError;
            if (absError > maxError) maxError = absError;
            errorSum += absError;
            errorCount++;
        }

        // Apply power
        flywheelMotor1.setPower(power);
        flywheelMotor2.setPower(power);

        // Telemetry
        double error = targetVelocity - avgVelocity;
        double avgError = errorCount > 0 ? errorSum / errorCount : 0;

        telemetry.addLine("=== SHOOTER FEEDFORWARD TUNER ===");
        telemetry.addLine();

        // Status
        if (kSFinderActive) {
            telemetry.addData("STATUS", "FINDING kS - Ramping power...");
            telemetry.addData("Current Power", String.format("%.4f", kSRampPower));
        } else if (flywheelEnabled) {
            telemetry.addData("STATUS", "RUNNING - Feedforward + PID");
        } else {
            telemetry.addData("STATUS", "STOPPED - Press A to start");
        }

        telemetry.addLine();
        telemetry.addLine("--- VELOCITY ---");
        telemetry.addData("Target", String.format("%.0f ticks/sec", targetVelocity));
        telemetry.addData("Actual", String.format("%.0f ticks/sec", avgVelocity));
        telemetry.addData("Error", String.format("%.1f ticks/sec", error));
        telemetry.addData("Max Observed", String.format("%.0f ticks/sec", maxVelocityObserved));

        telemetry.addLine();
        telemetry.addLine("--- POWER OUTPUT ---");
        telemetry.addData("Total Power", String.format("%.4f", power));
        telemetry.addData("Feedforward", String.format("%.4f", ffOutput));
        telemetry.addData("PID", String.format("%.4f", pidOutput));

        telemetry.addLine();
        telemetry.addLine("--- TUNING MODE: " + currentMode.name() + " ---");
        telemetry.addData("kS (static friction)", String.format("%.5f %s", ShooterFeedforwardConstants.kS, currentMode == TuningMode.kS ? "<<<" : ""));
        telemetry.addData("kV (velocity gain)", String.format("%.6f %s", ShooterFeedforwardConstants.kV, currentMode == TuningMode.kV ? "<<<" : ""));
        telemetry.addData("kI (integral)", String.format("%.6f %s", ShooterFeedforwardConstants.kI, currentMode == TuningMode.kI ? "<<<" : ""));
        telemetry.addData("kP (proportional)", String.format("%.6f %s", ShooterFeedforwardConstants.kP, currentMode == TuningMode.kP ? "<<<" : ""));

        telemetry.addLine();
        telemetry.addLine("--- ERROR STATS ---");
        telemetry.addData("Avg Error", String.format("%.1f", avgError));
        telemetry.addData("Min Error", String.format("%.1f", minError == Double.MAX_VALUE ? 0 : minError));
        telemetry.addData("Max Error", String.format("%.1f", maxError));

        boolean atTarget = Math.abs(error) < ShooterFeedforwardConstants.VELOCITY_TOLERANCE;
        telemetry.addData("AT TARGET?", atTarget ? "YES" : "NO");

        telemetry.addLine();
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("A:Toggle | B:Mode | DPAD:Adjust");
        telemetry.addLine("Y:Find kS | X:Reset | RB:Disturbance");

        telemetry.update();
    }

    private void handleInput() {
        if (buttonDebounce.seconds() < 0.2) return;

        // A: Toggle flywheel
        if (gamepad1.a) {
            flywheelEnabled = !flywheelEnabled;
            if (flywheelEnabled) {
                resetStats();
                updateControllers();
            }
            buttonDebounce.reset();
        }

        // B: Switch tuning mode
        if (gamepad1.b) {
            switch (currentMode) {
                case kS: currentMode = TuningMode.kV; break;
                case kV: currentMode = TuningMode.kI; break;
                case kI: currentMode = TuningMode.kP; break;
                case kP: currentMode = TuningMode.kS; break;
            }
            buttonDebounce.reset();
        }

        // Y: Start kS finder
        if (gamepad1.y) {
            kSFinderActive = true;
            kSRampPower = 0;
            kSRampTimer.reset();
            flywheelEnabled = false;
            buttonDebounce.reset();
        }

        // X: Reset to defaults
        if (gamepad1.x) {
            ShooterFeedforwardConstants.kS = 0.05;
            ShooterFeedforwardConstants.kV = 0.00035;
            ShooterFeedforwardConstants.kI = 0.0002;
            ShooterFeedforwardConstants.kP = 0.0001;
            updateControllers();
            resetStats();
            buttonDebounce.reset();
        }

        // Right Bumper: Simulate disturbance (brief power cut)
        if (gamepad1.right_bumper && flywheelEnabled) {
            flywheelMotor1.setPower(0);
            flywheelMotor2.setPower(0);
            resetStats();
            buttonDebounce.reset();
        }

        // DPAD: Adjust current parameter
        double increment = getIncrement();
        if (gamepad1.dpad_up) {
            adjustParameter(increment);
            updateControllers();
            buttonDebounce.reset();
        }
        if (gamepad1.dpad_down) {
            adjustParameter(-increment);
            updateControllers();
            buttonDebounce.reset();
        }

        // Left/Right DPAD: Adjust target velocity
        if (gamepad1.dpad_right) {
            ShooterFeedforwardConstants.TUNING_VELOCITY += 100;
            resetStats();
            buttonDebounce.reset();
        }
        if (gamepad1.dpad_left) {
            ShooterFeedforwardConstants.TUNING_VELOCITY -= 100;
            resetStats();
            buttonDebounce.reset();
        }
    }

    private double getIncrement() {
        switch (currentMode) {
            case kS: return 0.005;
            case kV: return 0.00001;
            case kI: return 0.00005;
            case kP: return 0.00005;
            default: return 0.001;
        }
    }

    private void adjustParameter(double delta) {
        switch (currentMode) {
            case kS:
                ShooterFeedforwardConstants.kS = Math.max(0, ShooterFeedforwardConstants.kS + delta);
                break;
            case kV:
                ShooterFeedforwardConstants.kV = Math.max(0, ShooterFeedforwardConstants.kV + delta);
                break;
            case kI:
                ShooterFeedforwardConstants.kI = Math.max(0, ShooterFeedforwardConstants.kI + delta);
                break;
            case kP:
                ShooterFeedforwardConstants.kP = Math.max(0, ShooterFeedforwardConstants.kP + delta);
                break;
        }
    }

    private void resetStats() {
        minError = Double.MAX_VALUE;
        maxError = 0;
        errorSum = 0;
        errorCount = 0;
        integralSum = 0;
        if (pidController != null) {
            pidController.reset();
        }
    }

    @Override
    public void stop() {
        flywheelMotor1.setPower(0);
        flywheelMotor2.setPower(0);
    }
}