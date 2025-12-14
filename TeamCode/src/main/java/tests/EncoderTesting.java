package tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Encoder", group = "Tests")
public class EncoderTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx shooterMotor = hardwareMap.get(DcMotorEx.class,"shooterMotor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx encoder = shooterMotor;


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.addData("Encoder Value:", encoder.getCurrentPosition());
            telemetry.addData("Velocity:", encoder.getVelocity());
            telemetry.addData("Velocity in RPM:", encoder.getVelocity(AngleUnit.RADIANS) * 60 * 10);
            telemetry.update();
        }
    }
}