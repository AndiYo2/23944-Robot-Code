package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import Constants.EnumConstants.IntakeState;
import utility.RobotHardware;

public class Intake extends SubsystemBase {
    RobotHardware robot;
    private IntakeState currentState = IntakeState.Idle;

    // Timed intake support
    private final ElapsedTime timedIntakeTimer = new ElapsedTime();
    private double timedIntakeDuration = 0;
    private boolean timedIntakeActive = false;

    @Override
    public void periodic() {
        // Check if timed intake should stop
        if (timedIntakeActive && timedIntakeTimer.seconds() >= timedIntakeDuration) {
            timedIntakeActive = false;
            stopIntake();
        }
        stateMachinePeriodic();
    }

    public Intake(){
        robot = RobotHardware.getInstance();
    }

    private void stateMachinePeriodic() {
        switch (currentState) {
            case Idle:
                // Motors stopped
                break;
            case Intaking:
                setIntakePower(1.0);
                setStagingMotorPower(1.0);
                break;
            case Reversing:
                setIntakePower(-1.0);
                setStagingMotorPower(-1.0);
                break;
            case StagingOnly:
                setIntakePower(0);
                setStagingMotorPower(1.0);
                break;
            case IntakeOnly:
                setIntakePower(1.0);
                setStagingMotorPower(0);
                break;
            case ReversedInBeltGo:
                setIntakePower(-1.0);
                setStagingMotorPower(1.0);
                break;
        }
    }

    public void setIntakePower(double power) {
        robot.intakeMotor.setPower(power);
    }
    public void setStagingMotorPower(double power) {
        robot.intakeBeltMotor.setPower(power);
    }
    public void runIntake(){
        currentState = IntakeState.Intaking;
    }

    /**
     * Start intake and automatically stop after duration.
     * Non-blocking - returns immediately, intake stops in periodic().
     */
    public void runIntakeBeltForDuration(double seconds) {
        currentState = IntakeState.ReversedInBeltGo;
        timedIntakeDuration = seconds;
        timedIntakeTimer.reset();
        timedIntakeActive = true;
    }
    public void stopIntake(){
        currentState = IntakeState.Idle;
        setIntakePower(0);
        setStagingMotorPower(0);
    }

    public void reverse(){
        currentState = IntakeState.Reversing;
    }
    public IntakeState getCurrentState() {
        return currentState;
    }


}
