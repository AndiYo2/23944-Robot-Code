package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import Constants.NamingConstants;
import Constants.ShooterConstants;

/**
 * Shooter PIDF Tuning OpMode.
 *
 * Uses PIDF values from ShooterConstants.ShooterPIDF.
 * Adjust values in RobotConstants and redeploy to test different settings.
 */
@TeleOp(name = "ShooterPIDF", group = "Tests")
public class ShooterPIDFTuningTeleOp extends OpMode {

    private DcMotorEx flywheelMotor1, flywheelMotor2;

    @Override
    public void init() {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter1);
        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, NamingConstants.Shooter.shooter2);
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Apply current PIDF from RobotConstants
        PIDFCoefficients pidf = new PIDFCoefficients(
            ShooterConstants.ShooterPIDF.P,
            ShooterConstants.ShooterPIDF.I,
            ShooterConstants.ShooterPIDF.D,
            ShooterConstants.ShooterPIDF.F
        );
        flywheelMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        flywheelMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        // Run at tuning velocity
        flywheelMotor1.setVelocity(ShooterConstants.ShooterPIDF.TUNING_VELOCITY);
        flywheelMotor2.setVelocity(ShooterConstants.ShooterPIDF.TUNING_VELOCITY);

        // Get current values for telemetry
        double currentVelocity1 = flywheelMotor1.getVelocity();
        double currentVelocity2 = flywheelMotor2.getVelocity();
        double error = ShooterConstants.ShooterPIDF.TUNING_VELOCITY - currentVelocity2;

        // Telemetry
        telemetry.addData("Target Velocity", ShooterConstants.ShooterPIDF.TUNING_VELOCITY);
        telemetry.addData("Actual Velocity 1", String.format("%.0f", currentVelocity1));
        telemetry.addData("Actual Velocity 2", String.format("%.0f", currentVelocity2));
        telemetry.addData("Error", String.format("%.1f", error));
        telemetry.addLine("-----------------------------");
        telemetry.addData("P", ShooterConstants.ShooterPIDF.P);
        telemetry.addData("I", ShooterConstants.ShooterPIDF.I);
        telemetry.addData("D", ShooterConstants.ShooterPIDF.D);
        telemetry.addData("F", ShooterConstants.ShooterPIDF.F);
        telemetry.update();
    }
}
