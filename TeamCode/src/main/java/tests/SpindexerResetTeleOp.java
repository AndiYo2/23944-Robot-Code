package tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import utility.RobotHardware;
import subsystems.Spindexer;

@TeleOp(name = "Spindexer Reset", group = "Tests")
public class SpindexerResetTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        Spindexer spindexer = new Spindexer();

        telemetry.addLine("Ready to reset spindexer");
        telemetry.addLine("Press A to reset to 0°");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            spindexer.periodic();

            // Press A to reset to 0
            if (gamepad1.a) {
                double currentPos = spindexer.getServoPosition();
                spindexer.rotateBy(-currentPos); // Rotate back to 0
            }

            telemetry.addData("Current Position", "%.1f°", spindexer.getServoPosition());
            telemetry.addData("Target Position", "%.1f°", spindexer.getTargetPosition());
            telemetry.addData("Rotating?", !spindexer.isDoneRotating());
            telemetry.addLine();
            telemetry.addLine("Press A to reset to 0°");
            telemetry.update();
        }
    }
}