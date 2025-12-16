package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import utility.RobotConstants;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotHardware;
import utility.ShootingStrategy;

public class Spindexer implements Subsystem {
    private final RobotHardware robot;
    FlickState flickState = FlickState.Idle;
    private double targetPosition;
    private double angleRange = 3;
    private boolean shouldRotate = false;
    private ShootingStrategy.Action[] shootingSequence = null;
    private int sequenceIndex = 0;

    // PID variables
    private double kP = 0.01;
    private double kI = 0.0;
    private double kD = 0.0;
    private double lastError = 0;
    private double integral = 0;
    private long lastTime = 0;

    public Spindexer() {
        this.robot = RobotHardware.getInstance();
        targetPosition = getServoPosition();
        lastTime = System.nanoTime();
    }

    public void flickBallOut() {
        flickState = FlickState.Start;
    }

    private void flipperPeriodic() {
        switch (flickState) {
            case Retracted:
                break;
            case Start:
                if (robot.spindexerFlipperServo.getPosition() == RobotConstants.Spindexer.FLIPPER_POSITION_EXTENDED) {
                    flickState = FlickState.Extended;
                    break;
                }
                robot.spindexerFlipperServo.setPosition(RobotConstants.Spindexer.FLIPPER_POSITION_EXTENDED);
                break;
            case Extended:
                if (robot.spindexerFlipperServo.getPosition() == RobotConstants.Spindexer.FLIPPER_POSITION_RETRACT) {
                    flickState = FlickState.Retracted;
                    break;
                }
                robot.spindexerFlipperServo.setPosition(RobotConstants.Spindexer.FLIPPER_POSITION_RETRACT);
                break;
        }
    }

    public FlickState getFlipperState() {
        return flickState;
    }

    public void rotate(double positionChange) {
        targetPosition += positionChange;
        // Wrap to [0, 360)
        while (targetPosition >= 360) targetPosition -= 360;
        while (targetPosition < 0) targetPosition += 360;
        shouldRotate = true;
        // Reset PID
        integral = 0;
        lastError = 0;
    }

    public double getServoPosition() {
        double pos = robot.spindexerEncoder.getVoltage();
        pos /= 3.3;
        pos *= 360;
        // Wrap to [0, 360)
        while (pos >= 360) pos -= 360;
        while (pos < 0) pos += 360;
        return pos;
    }

    public double getTargetPosition(){
        return targetPosition;
    }

    public void rotationUpdater() {
        double currentPosition = getServoPosition();
        double error = targetPosition - currentPosition;

        // Take shortest path
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // Are we close enough?
        if (Math.abs(error) < angleRange) {
            robot.spindexerServo.setPower(0);
            shouldRotate = false;
            integral = 0;
            return;
        }

        // Calculate dt
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        // PID calculations
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp power to [-1, 1]
        power = Math.max(-1, Math.min(1, power));

        robot.spindexerServo.setPower(power);
    }

    public boolean isDoneRotating() {
        double currentPosition = getServoPosition();
        double difference = targetPosition - currentPosition;

        // Take shortest path
        if (difference > 180) difference -= 360;
        if (difference < -180) difference += 360;

        return Math.abs(difference) < angleRange;
    }

    public void startShootingSequence(RobotConstants.MotiffPattern goalPattern) {
        shootingSequence = ShootingStrategy.getShootingSequence(
                robot.spindexerPattern,
                goalPattern
        );
        sequenceIndex = 0;
    }

    public ShootingStrategy.Action getNextAction() {
        if (shootingSequence != null && sequenceIndex < shootingSequence.length) {
            return shootingSequence[sequenceIndex];
        }
        return null;
    }

    public void completeCurrentAction() {
        if (shootingSequence != null) {
            sequenceIndex++;
            if (sequenceIndex >= shootingSequence.length) {
                shootingSequence = null;
                sequenceIndex = 0;
            }
        }
    }

    public boolean hasMoreActions() {
        return shootingSequence != null && sequenceIndex < shootingSequence.length;
    }

    @Override
    public void periodic() {
        flipperPeriodic();
        if(shouldRotate)
            rotationUpdater();
    }
}