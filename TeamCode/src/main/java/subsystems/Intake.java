package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import utility.RobotConstants.Enums.IntakeState;
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


    public double getIntakePower() {
        return robot.intakeMotor.getPower();
    }

    public void setIntakePower(double power) {
        robot.intakeMotor.setPower(power);
    }

    public void stopIntakeMotor() {
        robot.intakeMotor.setPower(0);
    }

    // ****** STAGING MOTOR ******


    public void setStagingMotorPower(double power) {
        robot.intakeBeltMotor.setPower(power);
    }

    public double getStagingMotorPower() {
        return robot.intakeBeltMotor.getPower();
    }

    public void stopStagingMotor() {
        robot.intakeBeltMotor.setPower(0);
    }

    // ****** INTAKE METHODS ******
    public void runIntake(){
        currentState = IntakeState.Intaking;
    }

    public void stopIntake(){
        currentState = IntakeState.Idle;
        setIntakePower(0);
        setStagingMotorPower(0);
    }

    public void reverseIntake() {
        currentState = IntakeState.Reversing;
    }

    public void runStagingOnly() {
        currentState = IntakeState.StagingOnly;
    }

    public void runIntakeOnly() {
        currentState = IntakeState.IntakeOnly;
    }

    // ****** STATE QUERIES ******
    public IntakeState getCurrentState() {
        return currentState;
    }

    public boolean isIdle() {
        return currentState == IntakeState.Idle;
    }


}
