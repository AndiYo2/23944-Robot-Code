package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.InternalSystems.StagingMotors;

public class IntakeMotors {

    private final double speed = 1;
    private boolean running = false;
    private DcMotorEx intakeMotor;
    private final StagingMotors stagingMotor = new StagingMotors();

    public void initIntake(HardwareMap hMap) {
        intakeMotor = hMap.get(DcMotorEx.class, "inMotor");
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void intakeBall(int positiveOrNeg) {
        intakeMotor.setPower(speed * positiveOrNeg);
    }

    public void stopIntakeBall() {
        intakeMotor.setPower(0);
    }
}