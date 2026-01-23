package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import Constants.EnumConstants.FieldState;
import Constants.FieldMap;
import Constants.RobotConstants;
import utility.RobotHardware;

public class Odometry extends SubsystemBase {
    // Hardware reference
    RobotHardware robot;

    // Field zone state - determines whether turret tracks goal or stays centered
    private FieldState fieldState = FieldState.IdleZone;


    public Odometry() {
        this.robot = RobotHardware.getInstance();
    }

    public void updateFieldState() {
        Pose2D currentPose = robot.pinpoint.getPosition();
        double centerX = currentPose.getX(DistanceUnit.INCH);
        double centerY = currentPose.getY(DistanceUnit.INCH);
        double heading = currentPose.getHeading(AngleUnit.RADIANS);
        double halfSize = RobotConstants.Robot.HALF_SIZE;

        // Robot corners in robot-relative coordinates [forward, right]
        double[][] corners = {
            { halfSize,  halfSize},  // Front-right
            { halfSize, -halfSize},  // Front-left
            {-halfSize,  halfSize},  // Back-right
            {-halfSize, -halfSize}   // Back-left
        };

        boolean inShootingZone = false;
        for (double[] corner : corners) {
            double cornerX = centerX + (corner[0] * Math.cos(heading) + corner[1] * Math.sin(heading));
            double cornerY = centerY + (corner[0] * Math.sin(heading) - corner[1] * Math.cos(heading));

            if (FieldMap.getPosition(cornerX, cornerY) == 'S') {
                inShootingZone = true;
                break;
            }
        }

        fieldState = inShootingZone ? FieldState.ShootingZone : FieldState.IdleZone;
    }

    public FieldState getFieldState() {
        return fieldState;
    }

    @Override
    public void periodic() {
        updateFieldState();
    }
}
