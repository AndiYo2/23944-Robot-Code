package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import utility.BallPattern;
import utility.RobotHardware;

public class Spindexer implements Subsystem {

    private final RobotHardware robot;
    private double goalRotationPosition = 0;   // Cumulative target position
    private boolean isAtGoal = true;

    private double motorPos = 0;

    public static BallPattern currentBallPattern;

    public Spindexer() {
        this.robot = RobotHardware.getInstance();
        currentBallPattern = new BallPattern();
    }


    public double getEncoderDegrees() {
        //Voltage / 3.3V * 360 Degrees
        motorPos = (robot.spindexerEncoder.getVoltage() / 3.3) * 360;
        return motorPos;
    }

    public void flickBallOut() {
        robot.spindexerServo.setPosition(.65);
        try {
            Thread.sleep(600);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        robot.spindexerServo.setPosition(0.25);
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
        setPositionAdd(120);
    }

    private double getAngularDistance(double angleA, double angleB) {
        double diff = Math.abs(angleA - angleB);
        return Math.min(diff, 360 - diff);
    }

    @Override
    public void periodic() {
        getEncoderDegrees();
        if (!isAtGoal) {
            robot.spindexerMotor.setPower(.5);

            // Normalize positions to 0-360 range
            double normalizedMotorPos = ((motorPos % 360) + 360) % 360;
            double normalizedGoalPos = ((goalRotationPosition % 360) + 360) % 360;

            // Check if within 5 degrees using the shortest path
            if (getAngularDistance(normalizedMotorPos, normalizedGoalPos) <= 2) {
                isAtGoal = true;
                robot.spindexerMotor.setPower(0);
            }
        }
    }

    public static boolean addBallLogic() {
        BallPattern.BallType ball = ColorSensorSubsytem.getBallColor();

        if (currentBallPattern.getBallInSlotX(1) == BallPattern.BallType.NONE) {
            currentBallPattern.addBallInSlotX(1, ball);
            return true;
        }
        return false;
    }
}