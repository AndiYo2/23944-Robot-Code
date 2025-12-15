package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import utility.RobotConstants;
import utility.RobotConstants.Enums.BallColor;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotHardware;
import utility.ShootingStrategy;

public class Spindexer implements Subsystem {

    private final RobotHardware robot;

    FlickState flickState = FlickState.Idle;

    private double position;
    private double targetPosition;
    private double power = 1;

    private double angleRange = 5;

    private PIDCoefficients pid;
    private double lastError = 0;
    private double integral = 0;
    private double lastTime;
    private boolean autoRotate = true;

    private boolean rotating = false;

    private ShootingStrategy.Action[] shootingSequence = null;
    private int sequenceIndex = 0;


    public Spindexer() {
        this.robot = RobotHardware.getInstance();
        position = getServoPosition();
        targetPosition = position;
        setPidCoeffs(RobotConstants.Spindexer.SPINDEXER_PID);
        lastTime = 0;
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
        // Wrap target position between 0-359
        targetPosition = ((targetPosition % 360) + 360) % 360;
    }

    public double getServoPosition() {
        double pos = robot.spindexerEncoder.getVoltage();
        pos /= 3.3;
        pos *= 360;
        return pos;
    }

    public double getTargetPosition(){
        return targetPosition;
    }

    public void rotationUpdater() {
        // Calculate shortest path
        double error = targetPosition - position;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // Calculate dt
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;
        lastTime = currentTime;

        // Update integral and derivative terms
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        // Calculate PID output
        double output = pid.p * error + pid.i * integral + pid.d * derivative;

        // Clamp output between -1 and 1
        output = Math.max(-1, Math.min(1, output));

        // Set motor power
        robot.spindexerServo.setPower(output);
    }

    public boolean isDoneRotating() {
        double error = Math.abs(targetPosition - position);
        error = Math.min(error, 360 - error);
        return error < angleRange;
    }

    public void setPidCoeffs(PIDCoefficients pid) {
        this.pid = pid;
        this.lastError = 0;
        this.integral = 0;
    }
    private void handleBallDetectionAndRotation() {
        // Check if ball entered slot 0 (intake position)
        if (ColorSensorSubsytem.ballJustEntered() && !rotating) {
            BallColor detectedColor = ColorSensorSubsytem.getBallColor();
            robot.spindexerPattern.setBallInSlotX(0, detectedColor);

            // Auto-rotate to make room for next ball
            rotate(120); // Move to next slot
            rotating = true;
        }

        // Handle rotation
        if (targetPosition != position) {
            rotationUpdater();
        }

        // Update state after rotation completes
        if (rotating && isDoneRotating()) {
            rotating = false;
            shiftPattern();
        }
    }

    private void shiftPattern() {
        BallColor temp = robot.spindexerPattern.getBallInSlotX(2);
        robot.spindexerPattern.setBallInSlotX(2, robot.spindexerPattern.getBallInSlotX(1));
        robot.spindexerPattern.setBallInSlotX(1, robot.spindexerPattern.getBallInSlotX(0));
        robot.spindexerPattern.setBallInSlotX(0, temp);
    }



    // Start the shooting sequence
    public void startShootingSequence(RobotConstants.MotiffPattern goalPattern) {
        shootingSequence = ShootingStrategy.getShootingSequence(
                robot.spindexerPattern,
                goalPattern
        );
        sequenceIndex = 0;
    }

    // Get the next action in the sequence
    public ShootingStrategy.Action getNextAction() {
        if (shootingSequence != null && sequenceIndex < shootingSequence.length) {
            return shootingSequence[sequenceIndex];
        }
        return null;
    }

    // Mark current action as complete and move to next
    public void completeCurrentAction() {
        if (shootingSequence != null) {
            sequenceIndex++;
            if (sequenceIndex >= shootingSequence.length) {
                shootingSequence = null;
                sequenceIndex = 0;
            }
        }
    }

    // Check if we have more actions to execute
    public boolean hasMoreActions() {
        return shootingSequence != null && sequenceIndex < shootingSequence.length;
    }


    @Override
    public void periodic() {
        position = getServoPosition();
        flipperPeriodic();
        if(targetPosition != position)
            rotationUpdater();

    }
}