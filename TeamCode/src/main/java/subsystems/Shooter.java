package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import utility.RobotHardware;

public class Shooter implements Subsystem {
        RobotHardware robot;

        public Shooter(){
            this.robot = RobotHardware.getInstance();
        }

        public double getShooterMotorVelocity() {
            return robot.shooterMotor.getVelocity(AngleUnit.RADIANS);
        }
        public void setShooterMotorVelocity(double speed) {
            robot.shooterMotor.setVelocity(speed, AngleUnit.RADIANS);
        }
        public double getShooterPower() {
            return robot.shooterMotor.getPower();
        }
        public void setShooterPower(double power) {
            robot.shooterMotor.setPower(power);
        }
        public void stopShooterMotor() {
            robot.shooterMotor.setPower(0);
        }



    // ****** STAGING MOTOR ******

    public void setStagingMotorSpeed(double speed) {
        robot.shooterBeltMotor.setVelocity(speed, AngleUnit.RADIANS);
    }

    public double getStagingMotorSpeed() {
        return robot.shooterBeltMotor.getVelocity(AngleUnit.RADIANS);
    }

    public void setStagingMotorPower(double power) {
        robot.shooterBeltMotor.setPower(power);
    }

    public double getStagingMotorPower() {
        return robot.shooterBeltMotor.getPower();
    }

    public void stopStagingMotor() {
        robot.shooterBeltMotor.setPower(0);
    }

    // ****** STAGING SERVO ******






}
