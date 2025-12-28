package tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import subsystems.MecanumDrive;
import utility.FieldMap;
import utility.RobotConstants;
import utility.RobotHardware;

@TeleOp (name = "FieldPositionTesting", group = "Tests")
public class PinpointTestTeleop extends CommandOpMode {
    protected MecanumDrive mecanumDrive;
    protected GamepadEx driverGamepad;
    private final RobotHardware robot = RobotHardware.getInstance();
    double robotX;
    double robotY;
    int robotHeading;




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

        mecanumDrive = new MecanumDrive();
        robotX = 56.5;
        robotY = 8.5;
        robotHeading = 90;
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
        robotX = robot.pinpoint.getPosX(DistanceUnit.INCH);
        robotY = robot.pinpoint.getPosY(DistanceUnit.INCH);
        robotHeading = (int) Math.round(robot.pinpoint.getHeading(AngleUnit.DEGREES));
    }

    private void updateTelemetry() {
        telemetry.addData("RobotPosition:", robot.pinpoint.getPosition());
        telemetry.addData("Robot X: ", "%.1f", robotX);
        telemetry.addData("Robot Y: ", "%.1f", robotY);
        telemetry.addData("Heading", "%d°", robotHeading);
        telemetry.addData("Robot Center Zone:", getRobotMapType());

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

        telemetry.update();
    }
}