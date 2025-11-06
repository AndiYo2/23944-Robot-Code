package org.firstinspires.ftc.teamcode.InternalSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class StagingServos {
    private static final double FLIPPER_EXTENDED = 0.9;
    private static final double FLIPPER_RETRACTED = 0.4;
    private static final double RAMP_EXTENDED = 1.0;
    private static final double RAMP_RETRACTED = 0.15;

    private HardwareMap hMap;
    private Servo flipperServo;
    private Servo rampServo;
    private final ElapsedTime runtime = new ElapsedTime();

    public void initStagingServos(HardwareMap hardwareMap) {
        hMap = hardwareMap;
        flipperServo = hMap.get(Servo.class, "flipperServo");
        rampServo = hMap.get(Servo.class, "rampServo");
    }

    public boolean flip() {
        runtime.reset();
        flipperServo.setPosition(FLIPPER_EXTENDED);
        while (runtime.seconds() < 0.5) {
            // Wait for servo to complete movement
        }
        flipperServo.setPosition(FLIPPER_RETRACTED);
        return true;
    }

    public boolean pushOutOfRamp() {
        runtime.reset();
        rampServo.setPosition(RAMP_EXTENDED);
        while (runtime.seconds() < 0.75) {
            // Wait for servo to complete movement
        }
        rampServo.setPosition(RAMP_RETRACTED);
        return true;
    }
}