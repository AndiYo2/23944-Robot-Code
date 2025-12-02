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
    public double getIntakeMotorVelocity() {
        return robot.intakeMotor.getVelocity(AngleUnit.RADIANS);
    }

    public void setIntakeMotorVelocity(double speed) {
        robot.intakeMotor.setVelocity(speed, AngleUnit.RADIANS);
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

    public void setStagingMotorSpeed(double speed) {
        robot.intakeBeltMotor.setVelocity(speed, AngleUnit.RADIANS);
    }

    public double getStagingMotorSpeed() {
        return robot.intakeBeltMotor.getVelocity(AngleUnit.RADIANS);
    }

    public void setStagingMotorPower(double power) {
        robot.intakeBeltMotor.setPower(power);
    }

    public double getStagingMotorPower() {
        return robot.intakeBeltMotor.getPower();
    }

    public void stopStagingMotor() {
        robot.intakeBeltMotor.setPower(0);
    }


    public void tryAddBallInSpindexer(){
        if(robot.spindexerMotor.getPower() == 0){
            robot.intakeBeltMotor.setPower(1);
            Spindexer.addBallLogic();
        }
    }
}
