package subsystems;

import android.graphics.Color;
import com.arcrobotics.ftclib.command.Subsystem;
import utility.BallPattern;
import utility.RobotHardware;

public class Spindexer implements Subsystem {

    private final RobotHardware robot;
    private double goalRotationPosition = 0;   // Cumulative target position
    private boolean isAtGoal = true;
    private double motorPos = 0;

    // State tracking for ball detection
    private BallPattern.BallType lastDetectedBall = BallPattern.BallType.NONE;
    private boolean hasRotatedForCurrentBall = false;

    public static BallPattern currentBallPattern;

    public Spindexer() {
        this.robot = RobotHardware.getInstance();
        currentBallPattern = new BallPattern();
    }

    public void flickBallOut() {
        robot.spindexerServo.setPosition(.65);
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        robot.spindexerServo.setPosition(0.25);
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
        setPositionAdd(120);
    }

    private double getAngularDistance(double angleA, double angleB) {
        double diff = Math.abs(angleA - angleB);
        return Math.min(diff, 360 - diff);
    }

    @Override
    public void periodic() {
        getEncoderDegrees();

        // Handle motor rotation to goal position
        if (!isAtGoal) {
            robot.spindexerMotor.setPower(-.2);

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

    public static boolean addBallLogic() {
        BallPattern.BallType ball = ColorSensorSubsytem.getBallColor();

        if (currentBallPattern.getBallInSlotX(1) == BallPattern.BallType.NONE) {
            currentBallPattern.setBallInSlotX(1, ball);
            return true;
        }
        return false;
    }

    public void startIntake() {
        robot.intakeBeltMotor.setPower(1);
        robot.intakeMotor.setPower(1);
    }

    public void stopIntake() {
        robot.intakeBeltMotor.setPower(0);
        robot.intakeMotor.setPower(0);
    }

    public void indexBalls() {
        // Start the intake motors
        startIntake();

        while (!currentBallPattern.isFull()) {
            BallPattern.BallType detectedBall = ColorSensorSubsytem.getBallColor();

            // Ball detected and we haven't rotated for it yet
            if (detectedBall != BallPattern.BallType.NONE && !hasRotatedForCurrentBall) {
                // Add ball to pattern
                currentBallPattern.setBallInSlotX(1, detectedBall);

                // Rotate to next slot
                rotate();

                // Mark that we've rotated for this ball
                hasRotatedForCurrentBall = true;
                lastDetectedBall = detectedBall;

                // Wait for rotation to complete before continuing
                while (!isAtGoal) {
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                        stopIntake();
                        return;
                    }
                }
            }
            // No ball detected - reset the flag so we can detect the next ball
            else if (detectedBall == BallPattern.BallType.NONE) {
                hasRotatedForCurrentBall = false;
            }

            // Small delay to prevent CPU spinning
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        // Stop intake when pattern is full or interrupted
        stopIntake();
    }

}