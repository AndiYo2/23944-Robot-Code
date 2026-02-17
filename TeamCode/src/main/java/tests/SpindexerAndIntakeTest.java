package tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Constants.EnumConstants;
import Constants.ShooterConstants;
import Constants.ShootingSequenceConstants;
import Constants.SpindexerConstants;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.RobotHardware;
import utility.SpindexerAndMotifStatus;

/**
 * Spindexer + Intake diagnostic TeleOp.
 *
 * CONTROLS:
 *   Right Bumper   = intake (hold to run, release to stop)
 *   Left Bumper    = reverse intake (hold)
 *   D-Pad Right    = rotate spindexer CW
 *   D-Pad Left     = rotate spindexer CCW
 *   X              = spindexer flipper (flick ball from slot 1 toward shooter)
 *   Right Trigger   = shooter flipper only (single flip, NOT full shoot macro)
 *   Y              = toggle shooting mode (Fast / Sorted)
 *   A              = reset spindexer to empty position + clear pattern
 *   D-Pad Up       = staging belt only (hold)
 *   D-Pad Down     = reverse belt, intake forward (hold) — ReversedInBeltGo
 */
@TeleOp(name = "Spindexer & Intake Test", group = "Tests")
public class SpindexerAndIntakeTest extends CommandOpMode {

    private Spindexer spindexer;
    private Intake intake;
    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driverGamepad;

    // Edge detection for button presses
    private boolean prevDpadRight = false;
    private boolean prevDpadLeft = false;
    private boolean prevX = false;
    private boolean prevY = false;
    private boolean prevA = false;

    // Shooter flipper state
    private boolean shooterFlipperExtended = false;
    private double shooterFlipperTimer = 0;
    private static final double SHOOTER_FLIP_DURATION = 0.2; // seconds

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        spindexer = new Spindexer();
        intake = new Intake();

        // Retract shooter flipper at start
        robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_RETRACT);

        register(spindexer, intake);
    }

    @Override
    public void run() {
        super.run();

        robot.clearBulkCache();
        robot.pollNextSensor();

        // ==================== SPINDEXER ROTATIONS ====================
        if (gamepad1.dpad_right && !prevDpadRight) {
            spindexer.rotateCW();
        }
        if (gamepad1.dpad_left && !prevDpadLeft) {
            spindexer.rotateCCW();
        }
        prevDpadRight = gamepad1.dpad_right;
        prevDpadLeft = gamepad1.dpad_left;

        // ==================== SPINDEXER FLIPPER ====================
        if (gamepad1.x && !prevX) {
            spindexer.triggerFlick();
        }
        prevX = gamepad1.x;

        // ==================== SHOOTER FLIPPER (single flip only) ====================
        if (gamepad1.right_trigger > 0.3 && !shooterFlipperExtended) {
            robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_EXTENDED);
            shooterFlipperExtended = true;
            shooterFlipperTimer = getRuntime();
        }
        if (shooterFlipperExtended && (getRuntime() - shooterFlipperTimer) > SHOOTER_FLIP_DURATION) {
            robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_RETRACT);
            shooterFlipperExtended = false;
        }

        // ==================== INTAKE ====================
        if (gamepad1.right_bumper) {
            intake.runIntake();
        } else if (gamepad1.left_bumper) {
            intake.reverse();
        } else if (gamepad1.dpad_up) {
            // Staging belt only
            intake.setIntakePower(0);
            intake.setStagingMotorPower(1.0);
        } else if (gamepad1.dpad_down) {
            // Reverse intake, belt forward
            intake.runReverseIntakeTransfer();
        } else {
            // Only stop if we were previously running via bumper/dpad hold
            if (intake.getCurrentState() != EnumConstants.IntakeState.Idle) {
                intake.stopIntake();
            }
        }

        // ==================== MODE TOGGLE ====================
        if (gamepad1.y && !prevY) {
            SpindexerConstants.currentMode =
                    (SpindexerConstants.currentMode == EnumConstants.ShootingMode.Fast)
                            ? EnumConstants.ShootingMode.Sorted
                            : EnumConstants.ShootingMode.Fast;
        }
        prevY = gamepad1.y;

        // ==================== RESET ====================
        if (gamepad1.a && !prevA) {
            spindexer.resetToEmptyPosition();
        }
        prevA = gamepad1.a;

        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addLine("=== SPINDEXER ===");
        telemetry.addData("Degrees", "%d deg", spindexer.getCurrentDegrees());
        telemetry.addData("Servo Position", "%.4f", spindexer.getServoPosition());
        telemetry.addData("Flipper State", spindexer.getCurrentState());
        telemetry.addData("Rotation Idle", spindexer.isRotationIdle());
        telemetry.addData("Ready to Flip", spindexer.isReadyToFlip());
        telemetry.addData("Shooting Mode", SpindexerConstants.currentMode);
        telemetry.addLine();

        // Ball pattern
        telemetry.addLine("=== BALL PATTERN ===");
        telemetry.addData("Pattern", SpindexerAndMotifStatus.SpindexerPattern.getSpindexerPatternString());
        telemetry.addData("Slot 0 (Intake)", SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(0));
        telemetry.addData("Slot 1 (Shooter)", SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(1));
        telemetry.addData("Slot 2 (Storage)", SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(2));
        telemetry.addData("Ball Count", SpindexerAndMotifStatus.SpindexerPattern.getBallCount());
        telemetry.addLine();

        // Intake
        telemetry.addLine("=== INTAKE ===");
        telemetry.addData("State", intake.getCurrentState());
        telemetry.addLine();

        // Shooter flipper
        telemetry.addLine("=== SHOOTER FLIPPER ===");
        telemetry.addData("Extended", shooterFlipperExtended);
        telemetry.addLine();

        // Sensor quick-check
        telemetry.addLine("=== SENSORS ===");
        telemetry.addData("Intake Sensor", robot.intakeSensorPair.quickCheck().color);
        telemetry.addData("Ramp Sensor", robot.rampSensorPair.quickCheck().color);
        telemetry.addData("Transfer Sensor", robot.transferSensorPair.quickCheck().color);
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("RB/LB: Intake/Reverse | DPad L/R: Rotate");
        telemetry.addLine("X: Spindexer Flip | RT: Shooter Flip");
        telemetry.addLine("Y: Toggle Mode | A: Reset Pattern");
        telemetry.addLine("DPad U: Belt Only | DPad D: Rev+Belt");

        telemetry.update();
    }
}
