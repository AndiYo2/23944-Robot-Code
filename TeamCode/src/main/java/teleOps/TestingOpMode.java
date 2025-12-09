package teleOps;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;



@TeleOp
public class TestingOpMode extends TeleOpTemplate {

    public int powerTicks = 0;

    @Override
    public void initialize() {
        initHardware(false);
        configureButtonBindings();
        shooter.disbaleCalcs = true;
        shooter.disableLimelight();
    }

    private void configureButtonBindings() {
        new Trigger(() -> gamepad1.left_trigger > 0.3)
                .whenActive(() -> intake.setIntakePower(1))
                .whenActive(() -> intake.setStagingMotorPower(1))
                .whenInactive(() -> intake.stopStagingMotor())
                .whenInactive(() -> intake.stopIntakeMotor());


        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(() -> shooter.flip());
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(() -> spindexer.rotate());
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(() -> spindexer.flickBallOut());
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(() -> spindexer.unstick());

        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> shooter.toggleLimelight());
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> shooter.disbaleCalcs = !shooter.disbaleCalcs);







    }

    @Override
    public void run() {
        super.run();
        if(gamepad1.dpad_right){

            shooter.setShooterPower(1);
        }
        if(gamepad1.left_stick_y > 0.3){
            if(powerTicks > 2){
                shooter.lowerRequiredVelocity();
                powerTicks = 0;
            }

        }
        else if(gamepad1.left_stick_y < -0.3){
            if(powerTicks > 2){
                shooter.raiseRequiredVelocity();
                powerTicks = 0;
            }
        }
        powerTicks++;

        if((gamepad1.right_stick_x > 0.3 || gamepad1.right_stick_x < -0.3) && shooter.disbaleCalcs){
            shooter.setTurretTurnerPower(gamepad1.right_stick_x);
        }else{
            shooter.setTurretTurnerPower(0);
        }




        telemetry.addData("Shooter Velocity:", shooter.getWheelVelocity());
        telemetry.addData("Shooter Power:", shooter.getRequiredVelocity());
        telemetry.addData("Shooter distance:", shooter.getDistanceToTarget());
        telemetry.addData("TurretStatus:", shooter.limelightDisabled());
        telemetry.addData("DistanceStatus:", shooter.disbaleCalcs);
        telemetry.addData("Ticks:", powerTicks);
        telemetry.addData("Spindexer Position:", spindexer.getEncoderDegrees());
        telemetry.update();
    }
}