package utility;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.*;
import subsystems.*;
import utility.RobotConstants.Enums.ShooterCases;

abstract public class TeleOpTemplate extends CommandOpMode {
        protected MecanumDrive mecanumDrive;
        protected Shooter shooter;
        protected Intake intake;
        protected Spindexer spindexer;
        protected ColorSensorSubsytem colorSensor;
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
            colorSensor = new ColorSensorSubsytem();
            spindexer = new Spindexer();
            register(intake, shooter, spindexer, colorSensor);

        }

        protected void startShoot(){
            if(shootCases == ShooterCases.Idle)
                shootCases = ShooterCases.Start;
        }
        protected void configureButtonBindings() {
            new Trigger(() -> gamepad1.left_trigger > 0.3)
                    .whenActive(() -> intake.runIntake())
                    .whenInactive(() -> intake.stopIntake());
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
            telemetry.addData("Shooter distance:", shooter.getDistanceToTarget());
            telemetry.addData("Color Detected:", colorSensor.getBallColor());
            telemetry.addData("Color String:", colorSensor.getColorDataString());
            telemetry.update();
        }
    }

