package teleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import subsystems.*;
import utility.RobotConstants;
import utility.RobotConstants.Enums.ShooterCases;
import utility.RobotHardware;

    abstract public class TeleOpTemplate extends CommandOpMode {
        protected MecanumDrive mecanumDrive;
        protected Shooter shooter;
        protected Intake intake;
        protected Spindexer spindexer;
        protected ColorSensorSubsytem colorSensorIntake;
        protected GamepadEx driverGamepad;
        private final RobotHardware robot = RobotHardware.getInstance();
        protected Limelight limelight;
        ShooterCases shootCases = ShooterCases.Idle;


        protected void initHardware(boolean isAuto) {
            driverGamepad = new GamepadEx(gamepad1);
            robot.init(hardwareMap, driverGamepad);
            mecanumDrive = new MecanumDrive();


            intake = new Intake();
            shooter = new Shooter();
            colorSensorIntake = new ColorSensorSubsytem();
            spindexer = new Spindexer();
            register(intake, shooter, spindexer,  colorSensorIntake);

        }

        protected void startShoot(){
            if(shootCases == ShooterCases.Idle)
                shootCases = ShooterCases.Start;
        }
        protected void configureButtonBindings() {
            new Trigger(() -> gamepad1.left_trigger > 0.3)
                    .whenActive(() -> intake.setIntakePower(1))
                    .whenActive(() -> intake.setStagingMotorPower(1))
                    .whenInactive(() -> intake.stopStagingMotor())
                    .whenInactive(() -> intake.stopIntakeMotor());
            new Trigger(() -> gamepad1.right_trigger > 0.3)
                    .whenActive(() -> startShoot());


            new GamepadButton(driverGamepad, GamepadKeys.Button.START)
                    .whenPressed(() -> mecanumDrive.resetYaw());

            new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                    .whenPressed(() -> mecanumDrive.toggleSlowMode());
            new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                    .whenPressed(() -> shooter.shootBall());
            new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                    .whenPressed(() -> spindexer.rotate());
            new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                    .whenPressed(() -> spindexer.flickBallOut());

            new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                    .whenPressed(() -> shooter.toggleLimelight());
            new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                    .whenPressed(() -> spindexer.unstick());

            new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                    .whenPressed(() -> shooter.lowerRequiredVelocity());
            new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                    .whenPressed(() -> shooter.raiseRequiredVelocity());





        }

        protected void shootingPeriodic(){

            switch (shootCases){
                case Idle:
                    break;
                case Start:
                    spindexer.flickBallOut();
                    shootCases = ShooterCases.SpindexerFlicking;
                    break;
                case SpindexerFlicking:
                    if(spindexer.getFlipperState() == RobotConstants.Enums.FlickState.Extended){
                        shootCases = ShooterCases.ShooterFlicking;
                        shooter.shootBall();
                    }
                    break;
                case ShooterFlicking:
                    if(spindexer.getFlipperState() == RobotConstants.Enums.FlickState.Retracted){
                        spindexer.rotate();
                        shootCases = ShooterCases.SpindexerRotating;
                    }
                    break;
                case SpindexerRotating:
                    if(spindexer.isDoneRotating()){
                        shootCases = ShooterCases.Idle;
                    }
                    break;
            }
        }

        public enum Alliance {
            RED,
            BLUE;
            public double adjust(double input) {
                return this == RED ? input : -input;
            }
        }

        @Override
        public void run() {
            super.run();

            mecanumDrive.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x);
            spindexer.periodic();
            shootingPeriodic();

            telemetry.addData("Shooter Power:", shooter.getRequiredVelocity());
            telemetry.addData("shooter distance:", shooter.getDistanceToTarget());
            telemetry.update();
        }
    }

