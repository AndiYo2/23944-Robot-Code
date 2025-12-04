package teleOps;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;


@TeleOp
public class FullManualTeleop extends OpModeTemplate {



    @Override
    public void initialize() {
        initHardware(false);
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new Trigger(() -> gamepad1.left_trigger > 0.3)
                .whenActive(() -> intake.setIntakePower(1))
                .whenActive(() -> intake.setStagingMotorPower(1))
                .whenInactive(() -> intake.stopStagingMotor())
                .whenInactive(() -> intake.stopIntakeMotor());

        new Trigger(() -> gamepad1.right_trigger > 0.3)
                .whenActive(() -> shooter.setShooterPower(1))
                .whenInactive(() -> shooter.stopShooterMotor());

        new GamepadButton(driverGamepad, GamepadKeys.Button.START)
                .whenPressed(() -> mecanumDrive.resetYaw());

        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(() -> mecanumDrive.toggleSlowMode());

        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(() -> shooter.flip());

        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(() -> spindexer.rotate());

        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(() -> spindexer.flickBallOut());

        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> shooter.toggleLimelight());



    }

    @Override
    public void run() {
        super.run();
        mecanumDrive.drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x);
        spindexer.periodic();
        colorSensorIntake.periodic();
        shooter.periodic();


        telemetry.addData("isRunning", true);
        //telemetry.addData("ColorSensor", colorSensorIntake.getColorDataString());
        telemetry.update();
    }
}