package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import utility.RobotHardware;

public class Intake implements Subsystem {
    RobotHardware robot;

    @Override
    public void periodic() {

    }
    public Intake(){
        robot = RobotHardware.getInstance();
    }


    public double getIntakePower() {
        return robot.intakeMotor.getPower();
    }

    public void setIntakePower(double power) {
        robot.intakeMotor.setPower(power);
    }

    public void stopIntakeMotor() {
        robot.intakeMotor.setPower(0);
    }

    // ****** STAGING MOTOR ******


    public void setStagingMotorPower(double power) {
        robot.intakeBeltMotor.setPower(power);
    }

    public double getStagingMotorPower() {
        return robot.intakeBeltMotor.getPower();
    }

    public void stopStagingMotor() {
        robot.intakeBeltMotor.setPower(0);
    }

    // ****** INTAKE METHODS ******
    public void runIntake(){
        setIntakePower(1);
        setStagingMotorPower(1);
    }
    public void stopIntake(){
        setIntakePower(0);
        setStagingMotorPower(0);
    }


}
