package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import utility.RobotConstants;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotHardware;

import static utility.RobotConstants.Shooter.FLICK_TIME;

public class Shooter implements Subsystem {


    RobotHardware robot;
    private double requiredVelocity = 1;
    FlickState currentState = FlickState.Idle;
    private ElapsedTime flickerTimer = new ElapsedTime();


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
        if(requiredVelocity > RobotConstants.Shooter.VELOCITY_ADJUSTMENT_STEP){
            requiredVelocity -= RobotConstants.Shooter.VELOCITY_ADJUSTMENT_STEP;
        }
    }
    public double getWheelVelocity(){
        return robot.shooterMotor1.getVelocity();
    }

    public void raiseRequiredVelocity(){
        requiredVelocity += RobotConstants.Shooter.VELOCITY_ADJUSTMENT_STEP;
    }

    public void setTurretTurnerPower(double power){
        robot.turretServo.setPower(power);
    }


    public double getDistanceToTarget() {

        return -1; // Invalid
    }


    private void stateMachinePeriodic() {
        if (currentState == FlickState.Idle) return; // Don't run unless activated

        switch (currentState) {
            case Idle:
                break;
            case Start:
                robot.shooterFlipper.setPosition(RobotConstants.Shooter.FLIPPER_POSITION_EXTENDED);
                flickerTimer.reset();
                currentState = FlickState.Extended;
                break;
            case Extended:
                if (flickerTimer.seconds() < FLICK_TIME) break;
                robot.shooterFlipper.setPosition(RobotConstants.Shooter.FLIPPER_POSITION_RETRACT);
                flickerTimer.reset();
                currentState = FlickState.Retracted;
                break;
            case Retracted:
                currentState = FlickState.Idle; // Return to idle after completion
                break;
        }
    }

    public void shootBall() {
        if (currentState == FlickState.Idle) { // Only start if not already running
            currentState = FlickState.Start;
        }
    }
    // ****** STATE QUERIES ******
    public FlickState getCurrentState(){
        return currentState;
    }

    public boolean isIdle() {
        return currentState == FlickState.Idle;
    }

    public void toggleLimelight(){RobotConstants.Limelight.isLimelightDisabled = !RobotConstants.Limelight.isLimelightDisabled;}



    @Override
    public void periodic() {
        robot.shooterMotor1.setPower(RobotConstants.Shooter.FULL_POWER);
        robot.shooterMotor2.setPower(RobotConstants.Shooter.FULL_POWER);
        stateMachinePeriodic();
        //Set back to FULL_POWER

        // Get Limelight data
        LLResult result = robot.limelight.getLatestResult();


    }
}