package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class IntakeMotors {

    // Declare private variables for the motor
    double speed = 0;
    boolean running = false;

    private DcMotor intakeMotor;


    public IntakeMotors() {
        // Constructor, can be left empty
    }

    public void grabBall(){
        intakeMotor.setPower(speed);
        if(checkBall()){
            //categorize with the color sensor and put it in a slot with the stuff
            //satvik go go power rangers code that stuff
        }
    }

    // Initialization method to map hardware
    public void init(HardwareMap hardwareMap) {
        // Retrieve and initialize motors from the hardware map

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        // Set motor directions based on configuration
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public boolean checkBall(){
        //if there is a ball, then we categorize it and do stuff
        return false;
    }

    public void changeIntakeStatus(){
        running = !running;
    }


}