package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import utility.RobotConstants;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotHardware;

public class Spindexer implements Subsystem {

    private final RobotHardware robot;
    private double goalRotationPosition = 0;   // Cumulative target position
    private boolean isAtGoal = true;
    private double motorPos = 0;
    int tempUnstick = 1;
    FlickState flickState = FlickState.Idle;

    // State tracking for ball detection



    public Spindexer() {
        this.robot = RobotHardware.getInstance();
    }



    public void flickBallOut() {
        flickState = FlickState.Start;
    }

    public FlickState getFlipperState(){
        return flickState;
    }

    private void flipperPeriodic(){
        switch(flickState){
            case Retracted:
                break;
            case Start:
                if(robot.spindexerServo.getPosition() == RobotConstants.Spindexer.FLIPPER_POSITION_EXTENDED){
                    flickState = FlickState.Extended;
                    break;
                }
                robot.spindexerServo.setPosition(RobotConstants.Spindexer.FLIPPER_POSITION_EXTENDED);
                break;
            case Extended:
                if(robot.spindexerServo.getPosition() == RobotConstants.Spindexer.FLIPPER_POSITION_RETRACT){
                    flickState = FlickState.Retracted;
                    break;
                }
                robot.spindexerServo.setPosition(RobotConstants.Spindexer.FLIPPER_POSITION_RETRACT);
                break;
        }
    }
    public double getEncoderDegrees() {
        //Voltage / 3.3V * 360 Degrees
        motorPos = (robot.spindexerEncoder.getVoltage() / 3.3) * 360;
        return motorPos;
    }

    public void setPositionAdd(double position) {
        goalRotationPosition += position;
        if(goalRotationPosition >= 360){
            goalRotationPosition = goalRotationPosition % 360 ;
        }else if(goalRotationPosition < 0){
            goalRotationPosition = 360 + goalRotationPosition;
        }
        isAtGoal = false;
    }

    public void rotate() {
        tempUnstick = 1;
        setPositionAdd(120);
    }

    public void unstick(){
        tempUnstick = -1;
        setPositionAdd(-120);
    }

    public boolean isDoneRotating(){return isAtGoal;}

    private double getAngularDistance(double angleA, double angleB) {
        double diff = Math.abs(angleA - angleB);
        return Math.min(diff, 360 - diff);
    }


    @Override
    public void periodic() {
        getEncoderDegrees();
        flipperPeriodic();

        // Handle motor rotation to goal position
        if (!isAtGoal) {
            robot.spindexerMotor.setPower(-.2 * tempUnstick);

            // Normalize positions to 0-360 range
            double normalizedMotorPos = ((motorPos % 360) + 360) % 360;
            double normalizedGoalPos = ((goalRotationPosition % 360) + 360) % 360;

            // Check if within 5 degrees using the shortest path
            if (getAngularDistance(normalizedMotorPos, normalizedGoalPos) <= 5) {
                isAtGoal = true;
                robot.spindexerMotor.setPower(0);
            }
        }
    }


}