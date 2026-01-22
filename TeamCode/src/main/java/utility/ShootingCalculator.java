package utility;

import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Constants.ShooterConstants;

/**
 * Centralized ballistic calculator for shoot-while-moving velocity compensation.
 *
 * This class handles all the math for compensating ball trajectory based on robot velocity:
 * - Radial velocity (toward/away from target) -> Adjusts exit velocity
 * - Tangential velocity (perpendicular to target) -> Adjusts turret lead angle
 * - Position prediction -> Accounts for ball exit delay
 * - Hood angle adjustment -> Compensates for changed effective velocity
 *
 * @see ShooterConstants for all tuning parameters
 */
public class ShootingCalculator {
    private final RobotHardware robot;

    // Smoothed velocity (EMA filter)
    private double smoothedVelX = 0;
    private double smoothedVelY = 0;

    // Calculated outputs
    private double distance;
    private double baseVelocity;
    private double baseHoodAngle;
    private double adjustedVelocity;
    private double adjustedHoodAngle;
    private double leadAngleDegrees;
    private double baseTurretAngleDegrees;  // Turret-relative angle to goal (before lead)
    private boolean safeToShoot = true;

    // Velocity components for telemetry
    private double vRadial = 0;
    private double vTangential = 0;
    private double velocityAdjustment = 0;

    // LUT instances for velocity and hood angle lookup
    private InterpLUT velocityLUT;
    private InterpLUT hoodLUT;

    public ShootingCalculator() {
        robot = RobotHardware.getInstance();
        initializeLUTs();
    }

    /**
     * Initialize InterpLUT instances from ShooterConstants data.
     */
    private void initializeLUTs() {
        // Velocity LUT
        velocityLUT = new InterpLUT();
        for (double[] entry : ShooterConstants.VELOCITY_DATA) {
            velocityLUT.add(entry[0], entry[1]);
        }
        velocityLUT.createLUT();

        // Hood LUT
        hoodLUT = new InterpLUT();
        for (double[] entry : ShooterConstants.HOOD_DATA) {
            hoodLUT.add(entry[0], entry[1]);
        }
        hoodLUT.createLUT();
    }

    /**
     * Update all calculations. Call this every loop.
     *
     * @param turretFieldPos Turret position on field [x, y] in inches
     * @param targetPos Goal position
     */
    public void update(double[] turretFieldPos, Pose targetPos) {
        // 1. Read and smooth velocity readings (EMA filter)
        double rawVelX = robot.pinpoint.getVelX(DistanceUnit.INCH);
        double rawVelY = robot.pinpoint.getVelY(DistanceUnit.INCH);

        smoothedVelX = ShooterConstants.VELOCITY_FILTER_ALPHA * rawVelX
                     + (1 - ShooterConstants.VELOCITY_FILTER_ALPHA) * smoothedVelX;
        smoothedVelY = ShooterConstants.VELOCITY_FILTER_ALPHA * rawVelY
                     + (1 - ShooterConstants.VELOCITY_FILTER_ALPHA) * smoothedVelY;

        // 2. Predict future position (where robot will be when ball exits)
        double predictedX = turretFieldPos[0] + smoothedVelX * ShooterConstants.BALL_EXIT_DELAY;
        double predictedY = turretFieldPos[1] + smoothedVelY * ShooterConstants.BALL_EXIT_DELAY;

        // 3. Calculate distance and angle from PREDICTED position to target
        double dx = targetPos.getX() - predictedX;
        double dy = targetPos.getY() - predictedY;
        distance = Math.sqrt(dx * dx + dy * dy);
        double angleToTargetRad = Math.atan2(dy, dx);

        // 3b. Calculate turret-relative angle (subtract robot heading from field angle)
        double robotHeadingRad = robot.pinpoint.getHeading(AngleUnit.RADIANS);
        double turretAngleRad = angleToTargetRad - robotHeadingRad;
        baseTurretAngleDegrees = Math.toDegrees(turretAngleRad);
        // Normalize to [-180, 180]
        baseTurretAngleDegrees = normalizeAngle(baseTurretAngleDegrees);

        // 4. Get base values from LUT (what we'd use if stationary)
        baseVelocity = velocityLUT.get(distance);
        baseHoodAngle = hoodLUT.get(distance);

        // 5. Decompose robot velocity relative to target direction
        double robotSpeed = Math.sqrt(smoothedVelX * smoothedVelX + smoothedVelY * smoothedVelY);
        double robotVelAngle = Math.atan2(smoothedVelY, smoothedVelX);
        double relativeAngle = angleToTargetRad - robotVelAngle;

        // Radial: positive = moving toward target, negative = moving away
        vRadial = robotSpeed * Math.cos(relativeAngle);
        // Tangential: positive = moving left relative to target line, negative = right
        vTangential = robotSpeed * Math.sin(relativeAngle);

        // 6. Apply radial compensation (velocity adjustment)
        // Moving toward target = ball already has that velocity = reduce flywheel speed
        // Moving away from target = ball loses that velocity = increase flywheel speed
        velocityAdjustment = -vRadial * ShooterConstants.RADIAL_VELOCITY_COEFFICIENT;
        adjustedVelocity = baseVelocity + velocityAdjustment;

        // 7. Calculate lead angle (tangential compensation)
        // Convert ball exit velocity from ticks/sec to inches/sec for proper angle calc
        double ballExitSpeedInches = baseVelocity * ShooterConstants.TICKS_TO_INCHES_PER_SEC;

        // Avoid division by zero
        if (ballExitSpeedInches > 1.0) {
            leadAngleDegrees = Math.toDegrees(Math.atan2(
                vTangential * ShooterConstants.TANGENTIAL_VELOCITY_COEFFICIENT,
                ballExitSpeedInches
            ));
        } else {
            leadAngleDegrees = 0;
        }

        // 8. Adjust hood angle for changed effective velocity
        // When we change velocity, the effective "distance" changes
        // Higher velocity = acts like shorter distance, lower velocity = longer distance
        if (adjustedVelocity > 0 && baseVelocity > 0) {
            double effectiveDistance = distance * (baseVelocity / adjustedVelocity);
            adjustedHoodAngle = hoodLUT.get(effectiveDistance);
        } else {
            adjustedHoodAngle = baseHoodAngle;
        }

        // Clamp hood angle to safe range
        adjustedHoodAngle = Math.max(ShooterConstants.HOOD_MIN_ANGLE,
                                    Math.min(ShooterConstants.HOOD_MAX_ANGLE, adjustedHoodAngle));

        // 9. Safety checks
        safeToShoot = checkSafetyBounds();
    }

    /**
     * Normalizes angle to [-180, 180] range.
     */
    private double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) degrees -= 360;
        else if (degrees < -180) degrees += 360;
        return degrees;
    }

    /**
     * Check if all calculated values are within safe operating bounds.
     */
    private boolean checkSafetyBounds() {
        // Check velocity adjustment magnitude
        if (Math.abs(velocityAdjustment) > ShooterConstants.MAX_VELOCITY_ADJUSTMENT) {
            return false;
        }

        // Check absolute velocity bounds
        if (adjustedVelocity < ShooterConstants.MIN_SAFE_VELOCITY) {
            return false;
        }
        if (adjustedVelocity > ShooterConstants.MAX_SAFE_VELOCITY) {
            return false;
        }

        // Check lead angle bounds
        if (Math.abs(leadAngleDegrees) > ShooterConstants.MAX_LEAD_ANGLE) {
            return false;
        }

        return true;
    }

    // ==================== GETTERS ====================

    /**
     * Get calculated distance to target (from predicted position).
     */
    public double getDistance() {
        return distance;
    }

    /**
     * Get base velocity from LUT (what we'd use if stationary).
     */
    public double getBaseVelocity() {
        return baseVelocity;
    }

    /**
     * Get adjusted velocity with radial compensation applied.
     */
    public double getAdjustedVelocity() {
        return adjustedVelocity;
    }

    /**
     * Get base hood angle from LUT (what we'd use if stationary).
     */
    public double getBaseHoodAngle() {
        return baseHoodAngle;
    }

    /**
     * Get adjusted hood angle compensating for velocity change.
     */
    public double getAdjustedHoodAngle() {
        return adjustedHoodAngle;
    }

    /**
     * Get calculated lead angle to add to turret aim.
     * Positive = aim left (leading a rightward strafe)
     * Negative = aim right (leading a leftward strafe)
     */
    public double getLeadAngleDegrees() {
        return leadAngleDegrees;
    }

    /**
     * Get base turret angle to goal (turret-relative, before lead angle).
     * This is the angle the turret should point if robot were stationary.
     * Normalized to [-180, 180] degrees.
     */
    public double getBaseTurretAngleDegrees() {
        return baseTurretAngleDegrees;
    }

    /**
     * Get final turret angle including lead compensation.
     * This is baseTurretAngle + leadAngle.
     */
    public double getFinalTurretAngleDegrees() {
        return baseTurretAngleDegrees + leadAngleDegrees;
    }

    /**
     * Check if current calculated values are safe for shooting.
     * Returns false if velocity or lead angle exceed safety limits.
     */
    public boolean isSafeToShoot() {
        return safeToShoot;
    }

    /**
     * Get smoothed X velocity (inches/sec, field-relative).
     */
    public double getSmoothedVelX() {
        return smoothedVelX;
    }

    /**
     * Get smoothed Y velocity (inches/sec, field-relative).
     */
    public double getSmoothedVelY() {
        return smoothedVelY;
    }

    /**
     * Get radial velocity component (toward/away from target).
     * Positive = moving toward target.
     */
    public double getRadialVelocity() {
        return vRadial;
    }

    /**
     * Get tangential velocity component (perpendicular to target).
     * Positive = moving left relative to target line.
     */
    public double getTangentialVelocity() {
        return vTangential;
    }

    /**
     * Get the velocity adjustment being applied (ticks/sec).
     */
    public double getVelocityAdjustment() {
        return velocityAdjustment;
    }

    /**
     * Get total robot speed (magnitude of velocity vector).
     */
    public double getRobotSpeed() {
        return Math.sqrt(smoothedVelX * smoothedVelX + smoothedVelY * smoothedVelY);
    }
}
