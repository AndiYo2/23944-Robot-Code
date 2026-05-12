package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Intake;
import utility.DualBallDetector;

/**
 * Runs the intake for a specified duration.
 * Automatically stops the intake when the duration has elapsed.
 * Optionally finishes early if a sensor gate detects a ball.
 */
public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final double duration;
    private final boolean reverseIntake;
    private final DualBallDetector sensorGate;
    private final ElapsedTime timer = new ElapsedTime();


    public IntakeCommand(Intake intake, double duration, boolean reverseIntake) {
        this(intake, duration, reverseIntake, null);
    }

    public IntakeCommand(Intake intake, double duration, boolean reverseIntake, DualBallDetector sensorGate) {
        this.intake = intake;
        this.duration = duration;
        this.reverseIntake = reverseIntake;
        this.sensorGate = sensorGate;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (reverseIntake) {
            intake.runReverseIntakeTransfer();
        } else {
            intake.runIntake();
        }
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= duration
                || (sensorGate != null && sensorGate.checkDistancePresent());
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
