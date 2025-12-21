package tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import subsystems.MecanumDrive;
import utility.FieldMap;
import utility.RobotHardware;

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
        updateLocation();
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
                return "Goal"; //SHOULD NOT HAPPEN!!!!!!!

        }
        return "";
    }
    public void updateLocation(){
        robotX = robot.pinpoint.getPosX(DistanceUnit.INCH);
        robotY = robot.pinpoint.getPosY(DistanceUnit.INCH);
        robotHeading = (int) Math.round(robot.pinpoint.getHeading(AngleUnit.DEGREES));
    }

    private void updateTelemetry() {
        telemetry.addData("RobotPosition:", robot.pinpoint.getPosition());
        telemetry.addData("Robot X: ", robotX);
        telemetry.addData("Robot Y: ", robotY);
        telemetry.addData("Heading", robotHeading);
        telemetry.addData("Robot Center type:", getRobotMapType());
        telemetry.update();
    }
}