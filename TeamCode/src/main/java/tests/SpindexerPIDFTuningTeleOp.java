package tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * DEPRECATED: Spindexer now uses position-controlled servo mode, not PIDF.
 * This tuning OpMode is no longer needed and has been stubbed out.
 *
 * The spindexer now uses an Axon servo in position mode with encoder verification.
 * No PIDF tuning is required - the servo handles positioning internally.
 */
@Disabled  // Disabled - Spindexer switched from CRServo+PIDF to position Servo
@TeleOp(name = "SpindexerPIDF", group = "Tests")
public class SpindexerPIDFTuningTeleOp extends OpMode {

    @Override
    public void init() {
        telemetry.addLine("This OpMode is deprecated.");
        telemetry.addLine("Spindexer now uses position servo mode.");
        telemetry.addLine("No PIDF tuning needed.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Stubbed - no longer functional
    }
}
