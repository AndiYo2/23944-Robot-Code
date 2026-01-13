package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import Constants.EnumConstants.IntakeState;
import utility.RobotHardware;

public class Intake implements Subsystem {
    RobotHardware robot;
    private IntakeState currentState = IntakeState.Idle;

    @Override
    public void periodic() {
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
    public void stopIntake(){
        currentState = IntakeState.Idle;
        setIntakePower(0);
        setStagingMotorPower(0);
    }
    public IntakeState getCurrentState() {
        return currentState;
    }


}
