package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import Constants.EnumConstants;
import Constants.EnumConstants.FlickState;
import Constants.FieldMap;
import Constants.LimelightConstants;
import Constants.RobotConstants;
import Constants.RobotHardware;
import Constants.ShooterConstants;

import static Constants.ShooterConstants.FLICK_TIME;

public class Shooter implements Subsystem {
    // Hardware reference
    RobotHardware robot;

    // Turret subsystem reference
    private Turret turret;

    // Odometry subsystem reference
    private Odometry odometry;

    // Flywheel velocity (ticks/sec) - updated each loop based on distance to goal
    private double requiredVelocity = ShooterConstants.DEFAULT_VELOCITY;

    // Ball flipper state machine
    FlickState currentState = FlickState.Idle;
    private ElapsedTime flickerTimer = new ElapsedTime();

    // Manual velocity override (for tuning/testing)
    private boolean manualVelocityMode = false;
    private double manualVelocity = ShooterConstants.DEFAULT_VELOCITY;


    public Shooter() {
        this.robot = RobotHardware.getInstance();

        // Configure flywheel PIDF for velocity control (use ShooterPIDF for unified tuning)
        com.qualcomm.robotcore.hardware.PIDFCoefficients pidCoefficients =
            new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                ShooterConstants.ShooterPIDF.P,
                ShooterConstants.ShooterPIDF.I,
                ShooterConstants.ShooterPIDF.D,
                ShooterConstants.ShooterPIDF.F
            );

        // Flywheel motor 1: velocity control mode
        robot.shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);

        // Flywheel motor 2: velocity control mode (reversed to spin same direction)
        robot.shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        robot.shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);

        // Start with ball flipper retracted
        robot.shooterFlipper.setPosition(ShooterConstants.FLIPPER_POSITION_RETRACT);
    }

    public void setTurret(Turret turret) {
        this.turret = turret;
    }

    public void setOdometry(Odometry odometry) {
        this.odometry = odometry;
    }

    public double getDistanceToTarget() {
        double[] turretPos = turret.getTurretFieldPosition();
        Pose goalPosition = FieldMap.getGoalPosition();

        double deltaX = goalPosition.getX() - turretPos[0];
        double deltaY = goalPosition.getY() - turretPos[1];
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    private double getVelocityFromDistance(double distance) {
        // Return fixed velocity during Limelight scan mode
        if (LimelightConstants.manuallySlowedForScan) {
            return ShooterConstants.FALLBACK_VELOCITY;
        }

        // Select correct lookup table based on alliance and zone
        boolean isBlue = (RobotConstants.Robot.allianceColor == EnumConstants.AllianceColor.Blue);
        double[][] table;

        if (isBlue) {
            table = (distance < ShooterConstants.BLUE_ZONE_BOUNDARY)
                ? ShooterConstants.BLUE_FRONT_LOOKUP
                : ShooterConstants.BLUE_BACK_LOOKUP;
        } else {
            table = (distance < ShooterConstants.RED_ZONE_BOUNDARY)
                ? ShooterConstants.RED_FRONT_LOOKUP
                : ShooterConstants.RED_BACK_LOOKUP;
        }

        return interpolateFromTable(table, distance);
    }

    private double interpolateFromTable(double[][] table, double distance) {
        // Clamp to table bounds
        if (distance <= table[0][0]) {
            return table[0][1];
        }
        int lastIndex = table.length - 1;
        if (distance >= table[lastIndex][0]) {
            return table[lastIndex][1];
        }

        // Linear interpolation between table entries
        for (int i = 0; i < table.length - 1; i++) {
            double dist1 = table[i][0];
            double dist2 = table[i + 1][0];

            if (distance >= dist1 && distance <= dist2) {
                double vel1 = table[i][1];
                double vel2 = table[i + 1][1];
                double ratio = (distance - dist1) / (dist2 - dist1);
                return vel1 + (vel2 - vel1) * ratio;
            }
        }

        return ShooterConstants.FALLBACK_VELOCITY;
    }

    private void updateVelocityFromDistance() {
        double distance = getDistanceToTarget();
        requiredVelocity = getVelocityFromDistance(distance);
    }

    private void flipperStateMachinePeriodic() {
        if (currentState == FlickState.Idle) return;

        switch (currentState) {
            case Start:
                robot.shooterFlipper.setPosition(ShooterConstants.FLIPPER_POSITION_EXTENDED);
                flickerTimer.reset();
                currentState = FlickState.Extended;
                break;
            case Extended:
                if (flickerTimer.seconds() < FLICK_TIME) break;
                robot.shooterFlipper.setPosition(ShooterConstants.FLIPPER_POSITION_RETRACT);
                flickerTimer.reset();
                currentState = FlickState.Retracted;
                break;
            case Retracted:
                currentState = FlickState.Idle;
                break;
        }
    }

    public void triggerShot() {
        if (currentState == FlickState.Idle) {
            currentState = FlickState.Start;
        }
    }

    public FlickState getCurrentState() {
        return currentState;
    }


    @Override
    public void periodic() {
        // Apply current PIDF from ShooterConstants each loop
        com.qualcomm.robotcore.hardware.PIDFCoefficients pidf =
            new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                ShooterConstants.ShooterPIDF.P,
                ShooterConstants.ShooterPIDF.I,
                ShooterConstants.ShooterPIDF.D,
                ShooterConstants.ShooterPIDF.F
            );
        robot.shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        robot.shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Set flywheel velocity
        double targetVelocity = manualVelocityMode ? manualVelocity : requiredVelocity;
        if (!manualVelocityMode) {
            updateVelocityFromDistance();
            targetVelocity = requiredVelocity;
        }
        robot.shooterMotor1.setVelocity(targetVelocity);
        robot.shooterMotor2.setVelocity(targetVelocity);

        // Run subsystem updates
        flipperStateMachinePeriodic();

        // Update turret with current field state from odometry
        if (turret != null && odometry != null) {
            turret.turretPeriodic(odometry.getFieldState());
        }
    }
}
