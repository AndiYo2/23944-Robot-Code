package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.geometry.Pose;

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
    // Velocity lookup table: [distance in inches, velocity]
    // < 24": 2100 | 24-86": 2200 | > 86": 2600
    private static final double[][] VELOCITY_LOOKUP = {
        {23.0, 2100.0},   // Very close range (< 24 inches)
        {24.0, 2200.0},   // Transition to mid range at 24 inches
        {88.0, 2300.0},   // Stay at 2200 through mid range
        {100.0, 2600.0}    // Far zone (> 86 inches)
    };

    RobotHardware robot;
    private double requiredVelocity = 2200; // Default to close range
    FlickState currentState = FlickState.Idle;
    public FieldState fieldState = FieldState.IdleZone;  // Public so other subsystems can access

    private ElapsedTime flickerTimer = new ElapsedTime();

    // Manual velocity override for tuning
    private boolean manualVelocityMode = false;
    private double manualVelocity = 2200;

    // Turret PID control variables
    private double targetTurretAngle = 0.0;
    private double lastTargetTurretAngle = 0.0; // Track last target to avoid unnecessary resets
    private double lastTurretError = 0;
    private double turretIntegral = 0;
    private long lastTurretTime = 0;

    // Turret PID coefficients (can be updated during tuning)
    private double turretKP = RobotConstants.Shooter.TURRET_PID.p;
    private double turretKI = RobotConstants.Shooter.TURRET_PID.i;
    private double turretKD = RobotConstants.Shooter.TURRET_PID.d;

    // Cumulative position tracking (needed because turret range exceeds 360°)
    private double lastRawTurretPosition = 0;
    private double cumulativeTurretPosition = 0;
    private int turretRotationCount = 0;

    // Limelight subsystem reference for dual-mode tracking
    private subsystems.Limelight limelightSubsystem;


    public Shooter() {
        this.robot = RobotHardware.getInstance();

        // Configure PIDF coefficients for velocity control
        com.qualcomm.robotcore.hardware.PIDFCoefficients pidCoefficients =
            new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                RobotConstants.Shooter.SHOOTER_P,
                RobotConstants.Shooter.SHOOTER_I,
                RobotConstants.Shooter.SHOOTER_D,
                RobotConstants.Shooter.SHOOTER_F
            );

        // Motor 1: Configure for velocity control with encoder
        robot.shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);

        // Motor 2: Configure for velocity control with encoder (reversed direction)
        robot.shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        robot.shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);

        lastTurretTime = System.nanoTime();

        // Initialize cumulative position tracking
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;

        // Normalize to [-180, 180]
        while (rawDegrees > 180) rawDegrees -= 360;
        while (rawDegrees < -180) rawDegrees += 360;

        lastRawTurretPosition = rawDegrees;
        turretRotationCount = 0;

        // Calculate initial cumulative position (encoder centered, no offset needed!)
        cumulativeTurretPosition = rawDegrees + (turretRotationCount * 360);

        // Initialize flipper to retracted position
        robot.shooterFlipper.setPosition(RobotConstants.Shooter.FLIPPER_POSITION_RETRACT);
    }


    public double getFlywheelPower() {
        return robot.shooterMotor1.getPower();
    }


    /**
     * Calculates the turret's position on the field
     * Accounts for turret offset from robot center
     * COORDINATE SYSTEM: +X forward, +Y LEFT (standard robot frame)
     * @return double array [turretX, turretY] in inches
     */
    private double[] getTurretFieldPosition() {
        Pose2D currentPose = robot.pinpoint.getPosition();
        double robotHeading = currentPose.getHeading(AngleUnit.RADIANS);

        // 2D rotation for Pedro Pathing coordinates (heading 90° = forward, 0° = right)
        // Robot-relative: OFFSET_X = right (4"), OFFSET_Y = forward (1")
        // Robot's right direction in field: (sin(θ), -cos(θ))
        // Robot's forward direction in field: (cos(θ), sin(θ))
        double turretX = currentPose.getX(DistanceUnit.INCH) +
                        (RobotConstants.Shooter.TURRET_OFFSET_X * Math.sin(robotHeading) +
                         RobotConstants.Shooter.TURRET_OFFSET_Y * Math.cos(robotHeading));
        double turretY = currentPose.getY(DistanceUnit.INCH) +
                        (-RobotConstants.Shooter.TURRET_OFFSET_X * Math.cos(robotHeading) +
                         RobotConstants.Shooter.TURRET_OFFSET_Y * Math.sin(robotHeading));

        return new double[]{turretX, turretY};
    }

    public double getDistanceToTarget() {
        double[] turretPos = getTurretFieldPosition();
        Pose goalPosition = FieldMap.getGoalPosition();

        double deltaX = goalPosition.getX() - turretPos[0];
        double deltaY = goalPosition.getY() - turretPos[1];
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Calculates required shooter velocity based on distance to goal
     * Uses linear interpolation between lookup table points
     * @param distance Distance to goal in inches
     * @return Required velocity in ticks per second
     */
    private double getVelocityFromDistance(double distance) {
        // Clamp to min/max if outside table range
        if (distance <= VELOCITY_LOOKUP[0][0]) {
            return VELOCITY_LOOKUP[0][1]; // Return minimum velocity
        }
        if (distance >= VELOCITY_LOOKUP[VELOCITY_LOOKUP.length - 1][0]) {
            return VELOCITY_LOOKUP[VELOCITY_LOOKUP.length - 1][1]; // Return maximum velocity
        }

        // Find the two points to interpolate between
        for (int i = 0; i < VELOCITY_LOOKUP.length - 1; i++) {
            double dist1 = VELOCITY_LOOKUP[i][0];
            double dist2 = VELOCITY_LOOKUP[i + 1][0];

            if (distance >= dist1 && distance <= dist2) {
                // Linear interpolation: velocity = v1 + (v2 - v1) * (d - d1) / (d2 - d1)
                double vel1 = VELOCITY_LOOKUP[i][1];
                double vel2 = VELOCITY_LOOKUP[i + 1][1];
                double ratio = (distance - dist1) / (dist2 - dist1);
                return vel1 + (vel2 - vel1) * ratio;
            }
        }

        // Fallback (shouldn't reach here)
        return 2200.0;
    }

    /**
     * Updates shooter velocity based on current distance to goal
     * Called automatically in periodic()
     */
    private void updateVelocityFromDistance() {
        double distance = getDistanceToTarget();
        requiredVelocity = getVelocityFromDistance(distance);
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
        // Check Limelight mode if subsystem is available
        if (limelightSubsystem != null &&
            limelightSubsystem.getCurrentMode() == RobotConstants.Enums.LimelightMode.TagTracking) {
            // Tag Tracking mode: Point at TAG_GOAL_POSITION
            setTurretDegree(getDegreesToTagGoal());
        } else {
            // Goal Tracking mode: Use field state logic
            switch (fieldState) {
                case IdleZone:
                    setTurretDegree(CENTER);
                    break;
                case ShootingZone:
                    setTurretDegree(getDegreesToGoal());
                    break;
            }
        }
    }
    
    public void setTurretDegree(double degree){
        setTurretAngle(degree);
    }
    
    public double getDegreesToGoal(){
        Pose2D currentPose = robot.pinpoint.getPosition();
        double[] turretPos = getTurretFieldPosition();
        Pose goalPosition = FieldMap.getGoalPosition();

        double deltaX = goalPosition.getX() - turretPos[0];
        double deltaY = goalPosition.getY() - turretPos[1];

        // Return angle relative to robot's heading (turret angle is robot-relative)
        double absoluteAngle = Math.atan2(deltaY, deltaX);
        double robotHeading = currentPose.getHeading(AngleUnit.RADIANS);
        double relativeAngle = Math.toDegrees(absoluteAngle - robotHeading);

        // Normalize to ±180° for shortest path
        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        // Apply tracking offset to compensate for systematic error
        relativeAngle += RobotConstants.Shooter.TURRET_TRACKING_OFFSET;

        return relativeAngle;
    }

    /**
     * Calculates turret angle to point at TAG_GOAL_POSITION (72, 143)
     * Used during Tag Tracking mode to position Limelight for tag detection
     * @return Turret angle in degrees (robot-relative)
     */
    public double getDegreesToTagGoal() {
        Pose2D currentPose = robot.pinpoint.getPosition();
        double[] turretPos = getTurretFieldPosition();

        double deltaX = RobotConstants.Limelight.TAG_GOAL_X - turretPos[0];
        double deltaY = RobotConstants.Limelight.TAG_GOAL_Y - turretPos[1];

        // Calculate angle relative to robot's heading
        double absoluteAngle = Math.atan2(deltaY, deltaX);
        double robotHeading = currentPose.getHeading(AngleUnit.RADIANS);
        double relativeAngle = Math.toDegrees(absoluteAngle - robotHeading);

        // Normalize to ±180° for shortest path
        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        // Apply tracking offset to compensate for systematic error
        relativeAngle += RobotConstants.Shooter.TURRET_TRACKING_OFFSET;

        return relativeAngle;
    }

    // ****** TURRET CONTROL METHODS ******

    /**
     * Gets the current servo position from the encoder with cumulative tracking
     * Tracks position across ±180° boundary to support full servo range (-360° to +270°)
     * @return Cumulative servo angle in degrees
     */
    public double getTurretPosition() {
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;

        // Normalize to [-180, 180] for boundary detection
        while (rawDegrees > 180) rawDegrees -= 360;
        while (rawDegrees < -180) rawDegrees += 360;

        // Track boundary crossings to maintain cumulative position
        double delta = rawDegrees - lastRawTurretPosition;

        // Detect crossing from +180 to -180 (clockwise rotation)
        if (delta < -180) {
            turretRotationCount++;
        }
        // Detect crossing from -180 to +180 (counter-clockwise rotation)
        else if (delta > 180) {
            turretRotationCount--;
        }

        lastRawTurretPosition = rawDegrees;

        // Calculate cumulative position (encoder now centered, no offset needed!)
        cumulativeTurretPosition = rawDegrees + (turretRotationCount * 360);

        return cumulativeTurretPosition; // Returns cumulative servo degrees
    }

    /**
     * Sets a new target angle for the turret
     * @param targetAngleTurret Target angle in TURRET degrees (45° CW, -60° CCW limits)
     */
    public void setTurretAngle(double targetAngleTurret) {
        // SAFETY: Clamp input to turret limits (45° CW, -60° CCW)
        targetAngleTurret = Math.max(-60.0, Math.min(45.0, targetAngleTurret));

        // Convert turret degrees to servo degrees
        double newTargetServo = targetAngleTurret * RobotConstants.Shooter.GEAR_RATIO;

        // Always update target to prevent drift (small changes matter for tracking)
        targetTurretAngle = newTargetServo;

        // Reset PID only when target changes significantly (> 0.5° servo)
        if (Math.abs(newTargetServo - lastTargetTurretAngle) > 0.5) {
            lastTargetTurretAngle = newTargetServo;
            turretIntegral = 0;
            lastTurretError = 0;
        }
    }

    /**
     * PID control loop for turret rotation
     * Called automatically in periodic()
     * Now always runs to prevent drift - no shouldRotateTurret flag needed!
     */
    public void turretRotationUpdater() {
        double currentPosition = getTurretPosition(); // In servo degrees

        // Calculate error (all in servo degrees)
        // NO wrapping - cumulative tracking allows us to reach any position directly
        double error = targetTurretAngle - currentPosition;

        // SAFETY: Hard limits in servo degrees
        // Turret limits: +45° CW, -60° CCW (in turret degrees)
        // → Servo limits: +270° CW, -360° CCW
        final double CW_LIMIT = 270.0;      // Maximum clockwise rotation (45° × 6)
        final double CCW_LIMIT = -360.0;    // Maximum counter-clockwise rotation (-60° × 6)
        final double LIMIT_MARGIN = 30.0;   // Start slowing down 30° before limit

        boolean nearCWLimit = currentPosition > (CW_LIMIT - LIMIT_MARGIN);
        boolean nearCCWLimit = currentPosition < (CCW_LIMIT + LIMIT_MARGIN);

        // If at hard limit and trying to go further, STOP IMMEDIATELY
        if ((currentPosition >= CW_LIMIT && error > 0) ||
            (currentPosition <= CCW_LIMIT && error < 0)) {
            robot.turretServo.setPower(0);
            turretIntegral = 0;  // FIX: Reset integral to prevent windup at limits
            return;
        }

        // Calculate dt
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTurretTime) / 1e9;
        lastTurretTime = currentTime;

        // Sanity check on dt
        if (dt > 1.0 || dt < 0.001) {
            dt = 0.02; // Default to 50Hz
        }

        // PID calculations with anti-windup
        turretIntegral += error * dt;
        turretIntegral = Math.max(-50, Math.min(50, turretIntegral)); // Clamp integral
        double derivative = (error - lastTurretError) / dt;
        lastTurretError = error;

        // Use instance variables for PID (can be updated during tuning)
        double power = (turretKP * error) + (turretKI * turretIntegral) + (turretKD * derivative);

        // SAFETY: Check for NaN/Infinite values
        if (!Double.isFinite(power)) {
            robot.turretServo.setPower(0);
            turretIntegral = 0;
            lastTurretError = 0;
            return;
        }

        // Reduce max power to save energy and reduce wear
        double maxPower = 0.5;

        // Further reduce power near limits
        if (nearCWLimit || nearCCWLimit) {
            maxPower = 0.3;
        }

        // Clamp power
        power = Math.max(-maxPower, Math.min(maxPower, power));

        // Deadband to prevent jitter and save battery when very close
        // PID still runs (tracks small changes) but we zero the output
        if (Math.abs(error) < RobotConstants.Shooter.ANGLE_RANGE) {
            power = 0;
        }

        robot.turretServo.setPower(power);
    }

    public double getTargetTurretAngle() {
        return targetTurretAngle;
    }

    public double getRequiredVelocity() {
        return requiredVelocity;
    }

    public void triggerShot() {
        if (currentState == FlickState.Idle) { // Only start if not already running
            currentState = FlickState.Start;
        }
    }
    // ****** STATE QUERIES ******
    public FlickState getCurrentState(){
        return currentState;
    }

    public void toggleLimelightEnabled(){
        RobotConstants.Limelight.isLimelightDisabled = !RobotConstants.Limelight.isLimelightDisabled;
    }

    /**
     * Sets the Limelight subsystem reference for dual-mode tracking
     * @param limelight The Limelight subsystem instance
     */
    public void setLimelightSubsystem(subsystems.Limelight limelight) {
        this.limelightSubsystem = limelight;
    }

    /**
     * Updates turret PID coefficients (for tuning mode)
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setTurretPID(double kP, double kI, double kD) {
        this.turretKP = kP;
        this.turretKI = kI;
        this.turretKD = kD;
    }

    /**
     * Enables manual velocity mode for tuning
     * When enabled, shooter uses manualVelocity instead of distance-based velocity
     * @param enabled Whether to enable manual velocity mode
     */
    public void setManualVelocityMode(boolean enabled) {
        this.manualVelocityMode = enabled;
    }

    /**
     * Sets the manual velocity target (only used when manual mode is enabled)
     * @param velocity Target velocity in ticks per second
     */
    public void setManualVelocity(double velocity) {
        this.manualVelocity = velocity;
    }

    /**
     * Gets the current manual velocity setting
     * @return Manual velocity in ticks per second
     */
    public double getManualVelocity() {
        return manualVelocity;
    }

    /**
     * Checks if manual velocity mode is active
     * @return true if manual velocity mode is enabled
     */
    public boolean isManualVelocityMode() {
        return manualVelocityMode;
    }

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
        // Update Pinpoint odometry data before using it
        robot.pinpoint.update();

        // Determine target velocity: manual mode (for tuning) or distance-based
        double targetVelocity;
        if (manualVelocityMode) {
            targetVelocity = manualVelocity;
        } else {
            // Update velocity based on distance to goal (from lookup table)
            updateVelocityFromDistance();
            targetVelocity = requiredVelocity;
        }

        // Both motors: Use velocity control with PIDF to maintain constant speed
        // This ensures the shooter stays at target velocity and recovers quickly after shooting
        robot.shooterMotor1.setVelocity(targetVelocity);
        robot.shooterMotor2.setVelocity(targetVelocity);

        flipperStateMachinePeriodic();
        updateFieldState(); // Update zone based on robot corner positions
        turretPeriodic(); // Set turret target based on field state
        turretRotationUpdater(); // Execute PID control to reach target
    }
}