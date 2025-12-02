package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.jetbrains.annotations.NotNull;
import utility.RobotHardware;

public class Limelight implements Subsystem {
    RobotHardware robot;

    private static final double kP = 10;
    private static final double kI = 0;
    private static final double kD = 0;

    private double targetX = 0;
    private double integral = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime targetLostTimer = new ElapsedTime();

    private static final double maxPower = 1;
    private static final double minPower = 0.3;
    private static final double positionTolerance = 2; //degrees
    private static final double targetLostTimeout = 0.5;
    private static final boolean invertMotor = false;

    private boolean targetWasVisible = false;

    //Use these for getTelemetryData() method
    private double lastTx = 0.0;
    private double currentError = 0.0;
    private double lastPIDOutput = 0.0;
    private double lastOutputPower = 0.0;
    private int currentTagId = -1;
    private int tagCount = 0;

    public Limelight() {
        robot = RobotHardware.getInstance();
    }

    public void start() {
        robot.limelight.start();
    }

    public void pause() {
        robot.limelight.pause();
    }

    public void stop() {
        if (robot.turretServo != null) {
            robot.turretServo.setPower(0);
        }
        if (robot.limelight != null) {
            robot.limelight.stop();
        }
    }

    public void getTarget() {
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();
            double dist = result.getBotposeAvgDist();
        }
    }

    public double getEncoderDegrees() {
        //Voltage / 3.3V * 360 Degrees
        double motorPos = (robot.shooterEncoder.getVoltage() / 3.3) * 360;
        return motorPos;
    }







    public void aimTurret() {
        LLResult result = robot.limelight.getLatestResult();
        tagCount = 0;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                targetWasVisible = true;
                targetLostTimer.reset();

                LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                double tx = fiducial.getTargetXDegrees();

                //For telemetry purposes
                lastTx = tx;
                tagCount = fiducials.size();
                currentTagId = fiducial.getFiducialId();

                double dt = timer.seconds();
                timer.reset();

                if (dt < 0.001) {
                    dt = 0.001;
                }
                if (dt > 1.0) {
                    dt = 1.0;
                }

                double error = tx - targetX;
                currentError = error;

                integral += error * dt;
                integral = Math.max(-50, Math.min(50, integral));

                double derivative = (error - lastError) / dt;

                double PIDOutput = (kP * error) + (kI * integral) + (kD * derivative);
                lastPIDOutput = PIDOutput;

                if (Math.abs(error) < positionTolerance) {
                    PIDOutput = 0;
                    integral = 0;
                }
                if (PIDOutput != 0) {
                    if (Math.abs(PIDOutput) < minPower) {
                        PIDOutput = minPower * Math.signum(PIDOutput);
                    }
                }

                double outputPower = Math.max(-maxPower, Math.min(maxPower, PIDOutput));
                lastOutputPower = outputPower;

                if (invertMotor) {
                    outputPower = -outputPower;
                }

                if (!Double.isFinite(outputPower)) {
                    outputPower = 0;
                    resetPID();
                }

                robot.turretServo.setPower(outputPower);
                lastError = error;
            } else {
                handleNoTargetFound();

                lastOutputPower = 0.0;
                lastPIDOutput = 0.0;
                currentError = 0.0;
                lastTx = 0.0;
                currentTagId = -1;
                tagCount = 0;
            }
        } else {
            handleNoTargetFound();

            lastOutputPower = 0.0;
            lastPIDOutput = 0.0;
            currentError = 0.0;
            lastTx = 0.0;
            currentTagId = -1;
            tagCount = 0;
        }

        if (tagCount == 0) {
            lastOutputPower = 0;
            lastPIDOutput = 0;
            currentError = 0;
        }
    }

    private void handleNoTargetFound() {
        if (targetWasVisible && targetLostTimer.seconds() < targetLostTimeout) {
            stopTurret();
        } else {
            stopTurret();
        }
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
    }

    private void stopTurret() {
        robot.turretServo.setPower(0);
        resetPID();
        targetWasVisible = false;
    }

    public String getTelemetryData() {
        double tx = lastTx;
        double error = currentError;
        double pidOutput = lastPIDOutput;
        double motorPower = lastOutputPower;
        int tagId = currentTagId;
        int tagsDetected = tagCount;

        boolean targetVisible = tagsDetected > 0;

        // Determine Status
        String status = targetVisible ? "LOCKED ON" : "TARGET LOST";

        // Determine Target Position
        String targetPosition;
        if (tx > positionTolerance) {
            targetPosition = "RIGHT";
        } else if (tx < -positionTolerance) {
            targetPosition = "LEFT";
        } else {
            targetPosition = "CENTER";
        }

        // Determine Motor Direction
        String motorDirection;
        if (motorPower > 0) {
            motorDirection = invertMotor ? "NEGATIVE (Actual)" : "POSITIVE (Actual)";
        } else if (motorPower < 0) {
            motorDirection = invertMotor ? "POSITIVE (Actual)" : "NEGATIVE (Actual)";
        } else {
            motorDirection = "STOPPED";
        }

        // Use StringBuilder or multiline string for formatting
        StringBuilder sb = new StringBuilder();

        // Match the requested format closely
        sb.append("Status: ").append(status).append("\n");
        sb.append(String.format("TX (degrees): %.2f\n", tx));
        sb.append("Target Position: ").append(targetPosition).append("\n");
        sb.append(String.format("Error: %.2f\n", error));
        sb.append(String.format("PID Output: %.3f\n", pidOutput));
        sb.append(String.format("Motor Power: %.3f\n", motorPower));
        sb.append("Motor Direction: ").append(motorDirection).append("\n");

        if (targetVisible) {
            sb.append("Tag ID: ").append(tagId).append("\n");
            sb.append("# Tags Detected: ").append(tagsDetected).append("\n");
        }

        // Optional debug
        sb.append(String.format("Target Lost Timer: %.2f s\n", targetLostTimer.seconds()));

        return sb.toString();
    }



    @Override
    public void periodic() {
        start();
        aimTurret();

    }
}