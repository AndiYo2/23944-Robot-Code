package teleOps;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import utility.RobotHardware;


@TeleOp
public class BlueTeleOp extends TeleOpTemplate {



    @Override
    public void initialize() {
        initHardware(false);
        configureButtonBindings();
        RobotHardware.getInstance().limelight.pipelineSwitch(3);
    }

    private int shotsQueued = 0;
    private boolean isShootingSequence = false;
    private boolean rightTriggerWasPressed = false;


    private void configureButtonBindings() {
        new Trigger(() -> gamepad1.left_trigger > 0.3)
                .whenActive(() -> intake.setIntakePower(1))
                .whenActive(() -> intake.setStagingMotorPower(1))
                .whenInactive(() -> intake.stopStagingMotor())
                .whenInactive(() -> intake.stopIntakeMotor());


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

        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> shooter.lowerRequiredVelocity());
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> shooter.raiseRequiredVelocity());
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> spindexer.unstick());




    }
    private void executeShootSequence() {
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> spindexer.flickBallOut()),
                new WaitCommand(200), // Wait for flickBallOut to complete
                new InstantCommand(() -> shooter.shootBall()),
                new WaitCommand(350),
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

        telemetry.addData("Shooter Power:", shooter.getRequiredVelocity());
        telemetry.addData("shooter distance:", shooter.getDistanceToTarget());
        telemetry.addData("LimelightStatus", shooter.limelightDisabled());
        //telemetry.addData("ColorSensor", colorSensorIntake.getColorDataString());
        telemetry.addData("ballType", "" + colorSensorIntake.getBallColor());
        telemetry.update();
    }
}