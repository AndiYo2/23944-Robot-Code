package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.hardware.DcMotor;
import utility.RobotConstants;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotHardware;

public class Shooter implements Subsystem {


    RobotHardware robot;
    private double requiredVelocity = 1;
    FlickState shootState = FlickState.Idle;


    public Shooter() {
        this.robot = RobotHardware.getInstance();;
        robot.shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double getShooterPower() {
        return robot.shooterMotor1.getPower();
    }
    public void setShooterPower(double power) {
        robot.shooterMotor1.setPower(power);
        robot.shooterMotor2.setPower(power);
    }
    public double getRequiredVelocity() {
        return requiredVelocity;
    }

    public void lowerRequiredVelocity(){
        if(requiredVelocity > .5){
            requiredVelocity -= .5;
        }
    }
    public double getWheelVelocity(){
        return robot.shooterMotor1.getVelocity();
    }

    public void raiseRequiredVelocity(){
        requiredVelocity += .5;
    }

    public void setTurretTurnerPower(double power){
        robot.turretServo.setPower(power);
    }


    public double getDistanceToTarget() {

        return -1; // Invalid
    }


    public void shootBall(){
        shootState = FlickState.Start;
    }


    private void flipperPeriodic(){
        switch(shootState){
            case Retracted:
                break;
            case Start:
                if(robot.shooterFlipper.getPosition() == RobotConstants.Shooter.FLIPPER_POSITION_EXTENDED){
                    shootState = FlickState.Extended;
                    break;
                }
                robot.shooterFlipper.setPosition(RobotConstants.Shooter.FLIPPER_POSITION_EXTENDED);
                break;
            case Extended:
                if(robot.shooterFlipper.getPosition() == RobotConstants.Shooter.FLIPPER_POSITION_RETRACT){
                    shootState = FlickState.Retracted;
                    break;
                }
                robot.shooterFlipper.setPosition(RobotConstants.Shooter.FLIPPER_POSITION_RETRACT);
                break;
        }
    }
    public FlickState getFlipperState(){
        return shootState;
    }

    public void toggleLimelight(){RobotConstants.Limelight.isLimelightDisabled = !RobotConstants.Limelight.isLimelightDisabled;}



    @Override
    public void periodic() {
        robot.shooterMotor1.setPower(1);
        robot.shooterMotor2.setPower(1);
        flipperPeriodic();
        //Set back to 1

        // Get Limelight data
        LLResult result = robot.limelight.getLatestResult();


    }
}