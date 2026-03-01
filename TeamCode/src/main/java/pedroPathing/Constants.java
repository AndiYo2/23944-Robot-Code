package pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.15,
                    0,
                    0.015,
                    0.02
            )).secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    .1,
                    0,
                    .01,
                    .015
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1,
                    0,
                    0.1,
                    0.01
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.07,
                    0,
                    0.0095,
                    0.6,
                    0.01))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.0075,
                    0,
                    0.00001,
                    0.6,
                    0.001))
            .lateralZeroPowerAcceleration(-62)
            .forwardZeroPowerAcceleration(-23.9)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(
                    0.05,
                    0.1,
                    0.001
            ))
            .translationalIntegral(0);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.2, .6);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .xVelocity(77)
            .yVelocity(57.7)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-0.5)
            .strafePodX(36.5)
            .distanceUnit(DistanceUnit.MM)
            .customEncoderResolution(19.65)
            .yawScalar(0.998148)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathBuilder pathBuilder(Follower follower) {
        return follower.pathBuilder();
    }

}