package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Limelight {

    private Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
    }

    public void switchPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public void pause() {
        limelight.pause();
    }

    public void start() {
        limelight.start();
    }

    public void stop(){
        limelight.stop();
    }
}
