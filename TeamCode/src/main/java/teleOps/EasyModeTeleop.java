package teleOps;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@TeleOp
public class EasyModeTeleop extends OpModeTemplate {

    private int shotsQueued = 0;
    private boolean isShootingSequence = false;
    private boolean rightTriggerWasPressed = false;



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
                .whenInactive(() -> intake.stopIntakeMotor())
                .whenInactive(() -> spindexer.indexBalls());

        new GamepadButton(driverGamepad, GamepadKeys.Button.START)
                .whenPressed(() -> mecanumDrive.resetYaw());

        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(() -> mecanumDrive.toggleSlowMode());

        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(() -> spindexer.rotate());

        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> shooter.toggleLimelight());
    }

    private void executeShootSequence() {
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> spindexer.flickBallOut()),
                new WaitCommand(100), // Wait for flickBallOut to complete
                new InstantCommand(() -> shooter.shootBall()),
                new WaitCommand(550),
                new InstantCommand(() -> spindexer.rotate()),
                new InstantCommand(() -> {
                    isShootingSequence = false;
                    shotsQueued--;
                }),
                new WaitCommand(200)
        ));
    }

    @Override
    public void run() {
        super.run();

        // Handle right trigger queueing
        boolean triggerPressed = gamepad1.right_trigger > 0.3;

        // Detect rising edge (button just pressed)
        if (triggerPressed && !rightTriggerWasPressed) {
            shotsQueued++;
        }
        rightTriggerWasPressed = triggerPressed;

        // Execute queued shots one at a time
        if (shotsQueued > 0 && !isShootingSequence) {
            isShootingSequence = true;
            executeShootSequence();
        }



        mecanumDrive.drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x);

        spindexer.periodic();
        colorSensorIntake.periodic();
        shooter.periodic();

        telemetry.addData("isRunning", true);
        telemetry.addData("Shots Queued", shotsQueued);
        telemetry.addData("Currently Shooting", isShootingSequence);
        telemetry.update();
    }
}