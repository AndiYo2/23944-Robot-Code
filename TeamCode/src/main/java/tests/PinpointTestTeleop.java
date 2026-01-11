package tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import subsystems.MecanumDrive;
import Constants.FieldMap;
import Constants.OdometryConstants;
import Constants.RobotConstants;
import Constants.RobotHardware;

@TeleOp (name = "FieldPositionTesting", group = "Tests")
public class PinpointTestTeleop extends CommandOpMode {
    protected MecanumDrive mecanumDrive;
    protected GamepadEx driverGamepad;
    private final RobotHardware robot = RobotHardware.getInstance();
    double robotX;
    double robotY;
    double robotHeading;




    @Override
    public void run() {
        super.run();

        updateLocation();
        updateDrivetrain();
        updateTelemetry();
    }

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        // Set starting position for field testing
        robot.pinpoint.setPosition(OdometryConstants.standardStartPoint);
        robot.pinpoint.update(); // Apply the position

        mecanumDrive = new MecanumDrive();
        robotX = 56.5;
        robotY = 8.5;
        robotHeading = Math.toRadians(90); // 90° = 1.571 radians
    }

    private void updateDrivetrain() {
        mecanumDrive.drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
    }

    private String getRobotMapType() {
        char positionType = FieldMap.getPosition(robotX, robotY);
        return getZoneName(positionType);
    }

    private String getZoneName(char positionType) {
        switch (positionType) {
            case 'N':
                return "Normal Field Spot";
            case 'S':
                return "Shoot Zone";
            case 'R':
                return "Red Zone";
            case 'B':
                return "Blue Zone";
            case 'G':
                return "Goal";
            default:
                return "Unknown";
        }
    }

    /**
     * Calculate corner positions in field coordinates
     * @return Array of 4 corners: [Front-Right, Front-Left, Back-Right, Back-Left]
     */
    private double[][] getCornerPositions() {
        double headingRad = Math.toRadians(robotHeading);
        double halfSize = RobotConstants.Robot.HALF_SIZE;

        // Define corners in robot frame (relative to center)
        double[][] robotFrameCorners = {
            { halfSize,  halfSize},  // Front-right
            { halfSize, -halfSize},  // Front-left
            {-halfSize,  halfSize},  // Back-right
            {-halfSize, -halfSize}   // Back-left
        };

        // Transform to field frame
        double[][] fieldCorners = new double[4][2];
        for (int i = 0; i < 4; i++) {
            fieldCorners[i][0] = robotX + (robotFrameCorners[i][0] * Math.cos(headingRad) -
                                           robotFrameCorners[i][1] * Math.sin(headingRad));
            fieldCorners[i][1] = robotY + (robotFrameCorners[i][0] * Math.sin(headingRad) +
                                           robotFrameCorners[i][1] * Math.cos(headingRad));
        }

        return fieldCorners;
    }
    public void updateLocation(){
        // CRITICAL: Update Pinpoint before reading values
        robot.pinpoint.update();

        robotX = robot.pinpoint.getPosX(DistanceUnit.INCH);
        robotY = robot.pinpoint.getPosY(DistanceUnit.INCH);
        robotHeading = robot.pinpoint.getHeading(AngleUnit.RADIANS);
    }

    private void updateTelemetry() {
        telemetry.addLine("========== POSITION ==========");
        telemetry.addData("Robot X", "%.1f inches", robotX);
        telemetry.addData("Robot Y", "%.1f inches", robotY);
        telemetry.addData("Heading", robotHeading);
        telemetry.addData("Center Zone", getRobotMapType());
        telemetry.addLine("");

        telemetry.addLine("========== CORNERS ==========");

        // Get all corner positions
        double[][] corners = getCornerPositions();
        String[] cornerNames = {"Front-Right", "Front-Left", "Back-Right", "Back-Left"};

        // Check if any corner is in shooting zone
        boolean inShootingZone = false;

        for (int i = 0; i < 4; i++) {
            double cornerX = corners[i][0];
            double cornerY = corners[i][1];
            char zone = FieldMap.getPosition(cornerX, cornerY);
            String zoneName = getZoneName(zone);

            // Add telemetry for this corner
            telemetry.addData(cornerNames[i], "(%.1f, %.1f) - %s", cornerX, cornerY, zoneName);

            // Check if in shooting zone
            if (zone == 'S') {
                inShootingZone = true;
            }
        }

        telemetry.addLine("=============================");
        telemetry.addData("In Shooting Zone?", inShootingZone ? "YES" : "NO");
        telemetry.addLine("");

        // IMU Status Info
        telemetry.addLine("========== IMU STATUS ==========");
        telemetry.addData("IMU Calibrated", "On Init (resetPosAndIMU)");
        telemetry.addData("Note", "Keep robot STILL during init!");
        telemetry.addLine("If heading drifts while stationary:");
        telemetry.addLine("  1. Restart OpMode with robot still");
        telemetry.addLine("  2. Check for vibration/movement");
        telemetry.addLine("  3. Move Pinpoint away from motors");

        telemetry.update();
    }
}