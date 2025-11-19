package OldSystems;

/*import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Limelight {

    private Limelight3A limelight;
    private IMU imu;
    private Telemetry telemetry;
    private double distance;

    public Limelight(HardwareMap hardwareMap, Telemetry tel, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);

        telemetry = tel;

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
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

    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botPose = result.getBotpose_MT2();
            distance = getDistance(result.getTa());
            telemetry.addData("Distance: ", distance);
            telemetry.addData("Tx: ", result.getTx());
            telemetry.addData("Ty: ", result.getTy());
            telemetry.addData("Ta: ", result.getTa());
            telemetry.addData("Bot pose: ", botPose);
        }
    }

    public double getDistance(double ta) {
        double scale = 0;
        distance = (scale / ta);
        return distance;
    }
}*/

