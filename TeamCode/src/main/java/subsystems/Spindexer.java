package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import utility.RobotConstants;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotHardware;

public class Spindexer implements Subsystem {

    private final RobotHardware robot;

    FlickState flickState = FlickState.Idle;

    public Spindexer() {
        this.robot = RobotHardware.getInstance();
    }
    public void flickBallOut() {
        flickState = FlickState.Start;
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

    public FlickState getFlipperState(){return flickState;}

    public void rotate() {
        robot.spindexerMotor.setTargetPosition(robot.spindexerMotor.getCurrentPosition() + 120);
        robot.spindexerMotor.set(1);
    }
    public void unstick(){
        robot.spindexerMotor.setTargetPosition(robot.spindexerMotor.getCurrentPosition() - 120);
        robot.spindexerMotor.set(1);
    }
    public boolean isDoneRotating(){return robot.spindexerMotor.atTargetPosition();}

    @Override
    public void periodic() {
        flipperPeriodic();
    }
}