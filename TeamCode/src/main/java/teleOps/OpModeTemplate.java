package teleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import subsystems.MecanumDrive;
import subsystems.Spindexer;
import subsystems.Intake;
import subsystems.Shooter;
import subsystems.ColorSensorSubsytem;
import utility.RobotHardware;

    abstract public class OpModeTemplate extends CommandOpMode {
        protected MecanumDrive mecanumDrive;
        protected Shooter shooter;
        protected Intake intake;
        protected Spindexer spindexer;
        protected ColorSensorSubsytem colorSensorIntake;
        protected GamepadEx driverGamepad;
        private final RobotHardware robot = RobotHardware.getInstance();

        protected void initHardware(boolean isAuto) {
            driverGamepad = new GamepadEx(gamepad1);
            robot.init(hardwareMap, driverGamepad);
            mecanumDrive = new MecanumDrive();


            intake = new Intake();
            shooter = new Shooter();
            colorSensorIntake = new ColorSensorSubsytem();
            spindexer = new Spindexer();
            register(intake, shooter, spindexer , colorSensorIntake);

        }

        public enum Alliance {
            RED,
            BLUE;
            public double adjust(double input) {
                return this == RED ? input : -input;
            }
        }
    }

