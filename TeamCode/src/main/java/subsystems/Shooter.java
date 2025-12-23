package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import utility.FieldMap;
import utility.RobotConstants;
import utility.RobotConstants.Enums.FieldState;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotHardware;

import static utility.RobotConstants.Shooter.CENTER;
import static utility.RobotConstants.Shooter.FLICK_TIME;

public class Shooter implements Subsystem {


    RobotHardware robot;
    private double requiredVelocity = 1;
    FlickState currentState = FlickState.Idle;
    public FieldState fieldState = FieldState.IdleZone;  // Public so other subsystems can access

    private ElapsedTime flickerTimer = new ElapsedTime();

    // Turret PID control variables
    private double targetTurretAngle = 0.0;
    private boolean shouldRotateTurret = false;
    private double lastTurretError = 0;
    private double turretIntegral = 0;
    private long lastTurretTime = 0;


    public Shooter() {
        this.robot = RobotHardware.getInstance();;
        robot.shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lastTurretTime = System.nanoTime();
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
        Pose2D currentPose = robot.pinpoint.getPosition();
        Pose goalPosition = FieldMap.getGoalPosition();

        // Account for turret offset from robot center
        double robotHeading = currentPose.getHeading(AngleUnit.RADIANS);
        double turretX = currentPose.getX(DistanceUnit.INCH) +
                        (RobotConstants.Shooter.TURRET_OFFSET_X * Math.cos(robotHeading) -
                         RobotConstants.Shooter.TURRET_OFFSET_Y * Math.sin(robotHeading));
        double turretY = currentPose.getY(DistanceUnit.INCH) +
                        (RobotConstants.Shooter.TURRET_OFFSET_X * Math.sin(robotHeading) +
                         RobotConstants.Shooter.TURRET_OFFSET_Y * Math.cos(robotHeading));

        double deltaX = goalPosition.getX() - turretX;
        double deltaY = goalPosition.getY() - turretY;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }


    private void flipperStateMachinePeriodic() {
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

    public void turretPeriodic(){
        switch (fieldState) {
            case IdleZone:
                setTurretDegree(CENTER);
                break;
            case ShootingZone:
                setTurretDegree(getDegreesToGoal());
                break;
        }
    }
    
    public void setTurretDegree(double degree){
        rotateTurret(degree);
    }
    
    public double getDegreesToGoal(){
        Pose2D currentPose = robot.pinpoint.getPosition();
        Pose goalPosition = FieldMap.getGoalPosition();

        // Account for turret offset from robot center
        double robotHeading = currentPose.getHeading(AngleUnit.RADIANS);
        double turretX = currentPose.getX(DistanceUnit.INCH) +
                        (RobotConstants.Shooter.TURRET_OFFSET_X * Math.cos(robotHeading) -
                         RobotConstants.Shooter.TURRET_OFFSET_Y * Math.sin(robotHeading));
        double turretY = currentPose.getY(DistanceUnit.INCH) +
                        (RobotConstants.Shooter.TURRET_OFFSET_X * Math.sin(robotHeading) +
                         RobotConstants.Shooter.TURRET_OFFSET_Y * Math.cos(robotHeading));

        double deltaX = goalPosition.getX() - turretX;
        double deltaY = goalPosition.getY() - turretY;

        // Return angle relative to robot's heading (turret angle is robot-relative)
        double absoluteAngle = Math.atan2(deltaY, deltaX);
        return Math.toDegrees(absoluteAngle - robotHeading);
    }

    // ****** TURRET CONTROL METHODS ******

    /**
     * Gets the current turret position from the encoder
     * @return Turret angle in degrees, centered at 0 with range ±360°
     */
    public double getTurretPosition() {
        double voltage = robot.turretEncoder.getVoltage();
        double degrees = (voltage / 3.3) * 360.0;

        // Convert from 0-360 range to ±180 range centered at encoder offset
        degrees -= RobotConstants.Shooter.ENCODER_OFFSET;

        // Normalize to [-180, 180]
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;

        return degrees;
    }

    /**
     * Sets a new target angle for the turret
     * @param targetAngle Target angle in degrees (±MAX_TURRET_ANGLE from center)
     */
    public void rotateTurret(double targetAngle) {
        // SAFETY: Clamp to maximum safe angle to prevent hardware damage
        targetTurretAngle = Math.max(-RobotConstants.Shooter.MAX_TURRET_ANGLE,
                                     Math.min(RobotConstants.Shooter.MAX_TURRET_ANGLE, targetAngle));
        shouldRotateTurret = true;

        // Reset PID
        turretIntegral = 0;
        lastTurretError = 0;
    }

    /**
     * PID control loop for turret rotation
     * Called automatically in periodic()
     */
    public void turretRotationUpdater() {
        if (!shouldRotateTurret) return;

        double currentPosition = getTurretPosition();

        // SAFETY: Emergency stop if position exceeds safe limits
        // This is a fail-safe in case encoder wrapping causes issues
        if (Math.abs(currentPosition) > RobotConstants.Shooter.MAX_TURRET_ANGLE + 10) {
            robot.turretServo.setPower(0);
            shouldRotateTurret = false;
            turretIntegral = 0;
            // Note: In a real scenario, you might want to log an error or alert the driver
            return;
        }

        double error = targetTurretAngle - currentPosition;

        // Take shortest path (wrap around if needed)
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // Check if we're close enough
        if (Math.abs(error) < RobotConstants.Shooter.ANGLE_RANGE) {
            robot.turretServo.setPower(0);
            shouldRotateTurret = false;
            turretIntegral = 0;
            return;
        }

        // Calculate dt
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTurretTime) / 1e9;
        lastTurretTime = currentTime;

        // PID calculations
        turretIntegral += error * dt;
        double derivative = (error - lastTurretError) / dt;
        lastTurretError = error;

        double kP = RobotConstants.Shooter.TURRET_PID.p;
        double kI = RobotConstants.Shooter.TURRET_PID.i;
        double kD = RobotConstants.Shooter.TURRET_PID.d;

        double power = (kP * error) + (kI * turretIntegral) + (kD * derivative);

        // Clamp power to [-1, 1]
        power = Math.max(-1, Math.min(1, power));

        robot.turretServo.setPower(power);
    }

    /**
     * Checks if the turret has reached its target position
     * @return true if at target, false otherwise
     */
    public boolean isDoneTurretRotating() {
        double currentPosition = getTurretPosition();
        double difference = targetTurretAngle - currentPosition;

        // Take shortest path
        if (difference > 180) difference -= 360;
        if (difference < -180) difference += 360;

        return Math.abs(difference) < RobotConstants.Shooter.ANGLE_RANGE;
    }

    public double getTargetTurretAngle() {
        return targetTurretAngle;
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

    // ****** ZONE DETECTION ******

    /**
     * Updates the field state based on robot position
     * Checks all four corners of the robot to determine if any are in the shooting zone
     */
    public void updateFieldState() {
        Pose2D currentPose = robot.pinpoint.getPosition();
        double centerX = currentPose.getX(DistanceUnit.INCH);
        double centerY = currentPose.getY(DistanceUnit.INCH);
        double heading = currentPose.getHeading(AngleUnit.RADIANS);

        // Calculate offsets for corners based on robot heading
        // Robot is square, so corners are at ±HALF_SIZE in both X and Y (robot frame)
        double halfSize = RobotConstants.Robot.HALF_SIZE;

        // Define corners in robot frame (relative to center)
        double[][] corners = {
            { halfSize,  halfSize},  // Front-right corner
            { halfSize, -halfSize},  // Front-left corner
            {-halfSize,  halfSize},  // Back-right corner
            {-halfSize, -halfSize}   // Back-left corner
        };

        // Check if any corner is in shooting zone
        boolean inShootingZone = false;

        for (double[] corner : corners) {
            // Transform corner from robot frame to field frame
            double cornerX = centerX + (corner[0] * Math.cos(heading) - corner[1] * Math.sin(heading));
            double cornerY = centerY + (corner[0] * Math.sin(heading) + corner[1] * Math.cos(heading));

            // Get zone character at this corner position
            char zone = FieldMap.getPosition(cornerX, cornerY);

            // Check if this corner is in shooting zone
            if (zone == 'S') {
                inShootingZone = true;
                break; // No need to check other corners
            }
        }

        // Update field state based on corner positions
        if (inShootingZone) {
            fieldState = FieldState.ShootingZone;
        } else {
            fieldState = FieldState.IdleZone;
        }
    }

    public FieldState getFieldState() {
        return fieldState;
    }

    @Override
    public void periodic() {
        robot.shooterMotor1.setPower(RobotConstants.Shooter.FULL_POWER);
        robot.shooterMotor2.setPower(RobotConstants.Shooter.FULL_POWER);
        flipperStateMachinePeriodic();
        turretRotationUpdater();
        updateFieldState(); // Update zone based on robot corner positions
        //Set back to FULL_POWER

        // Get Limelight data
        LLResult result = robot.limelight.getLatestResult();


    }
}