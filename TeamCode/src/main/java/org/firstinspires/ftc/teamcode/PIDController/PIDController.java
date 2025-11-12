package org.firstinspires.ftc.teamcode.PIDController;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private final VoltageSensor batteryVoltageSensor;
    private double kp, ki, kd, integralSum, lastError;
    private final double targetVoltage = 13.75;

    ElapsedTime timer = new ElapsedTime();

    public PIDController(double kp, double ki, double kd, HardwareMap hardwareMap) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    //Returns a value from 0 to 1; the returned value will be the power the motors are set to
    public double controlVoltage() {
        double currentVoltage = batteryVoltageSensor.getVoltage();
        double error = targetVoltage - currentVoltage;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();

        double output = (error * kp) + (derivative * kd) + (integralSum * ki);

        lastError = error;
        timer.reset();

        return output;
    }
}
