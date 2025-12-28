package utility;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Intake;
import subsystems.Spindexer;

/**
 * Coordinates jam clearance between Spindexer and Intake.
 * When spindexer detects a stall, runs intake forward briefly to push ball through.
 */
public class SpindexerJamClearance implements Spindexer.JamClearanceCallback {
    private final Intake intake;
    private final Spindexer spindexer;

    private enum ClearanceState {
        IDLE,
        CLEARING
    }

    private ClearanceState state = ClearanceState.IDLE;
    private ElapsedTime clearanceTimer = new ElapsedTime();
    private boolean wasIntakeRunningBeforeJam = false;

    public SpindexerJamClearance(Intake intake, Spindexer spindexer) {
        this.intake = intake;
        this.spindexer = spindexer;
        spindexer.setJamClearanceCallback(this);
    }

    @Override
    public void onJamDetected() {
        // Only clear jam if spindexer actually has balls
        RobotHardware robot = RobotHardware.getInstance();
        boolean hasBalls = robot.spindexerPattern.getBallInSlotX(0) != RobotConstants.Enums.BallColor.None ||
                           robot.spindexerPattern.getBallInSlotX(1) != RobotConstants.Enums.BallColor.None ||
                           robot.spindexerPattern.getBallInSlotX(2) != RobotConstants.Enums.BallColor.None;

        if (!hasBalls) {
            return; // No balls present, ignore jam detection
        }

        wasIntakeRunningBeforeJam = !intake.isIdle();
        state = ClearanceState.CLEARING;
        clearanceTimer.reset();
        intake.runIntake(); // Forward to push ball through
    }

    @Override
    public boolean isJamClearingInProgress() {
        return state != ClearanceState.IDLE;
    }

    public void periodic() {
        if (state == ClearanceState.CLEARING) {
            if (clearanceTimer.seconds() >= RobotConstants.Spindexer.INTAKE_CLEARING_TIME) {
                // Restore previous intake state
                if (!wasIntakeRunningBeforeJam) {
                    intake.stopIntake();
                }
                state = ClearanceState.IDLE;
            }
        }
    }

    public String getStatus() {
        if (state == ClearanceState.CLEARING) {
            return String.format("CLEARING JAM (%.2fs)",
                RobotConstants.Spindexer.INTAKE_CLEARING_TIME - clearanceTimer.seconds());
        }
        return "Ready";
    }
}
