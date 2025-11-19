package teleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import subsystems.MecanumDrive;
import subsystems.Spindexer;
import subsystems.Intake;
import subsystems.Shooter;
import subsystems.ColorSensor;
import subsystems.Limelight;

    abstract public class OpModeTemplate extends CommandOpMode {
        protected MecanumDrive mecanumDrive;
        protected Shooter shooter;
        protected Intake intake;
        protected Spindexer spindexer;
        protected ColorSensor colorSensor;
        protected GamepadEx driverGamepad;

        protected void initHardware(boolean isAuto) {
            mecanumDrive = new MecanumDrive();


            intake = new Intake();
            shooter = new Shooter();
            colorSensor = new ColorSensor();
            spindexer = new Spindexer();
            register(intake, shooter, colorSensor, spindexer);
            driverGamepad = new GamepadEx(gamepad1);
        }

        public enum Alliance {
            RED,
            BLUE;
            public double adjust(double input) {
                return this == RED ? input : -input;
            }
        }
    }

