package pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    static double yVelocity =72.241424480084276,
            xVelocity = 83.29543178288017,
            robotMass = 15;




    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(robotMass)
        .forwardZeroPowerAcceleration(-33.661051028806696)
        .lateralZeroPowerAcceleration(-62.9508309274868)
    .translationalPIDFCoefficients(new PIDFCoefficients(
            0.1,
            0,
            0.0125,
            0.015
))
        .headingPIDFCoefficients(new PIDFCoefficients(
                1,
                0,
                0.06,
                0.02
))
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                0.01,
                0,
                0.00001,
                0.6,
                0.01
                //MAYBE SWITCH IF BROKEN
))
        ;
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(xVelocity)
            .yVelocity(yVelocity);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)  // Centered by design in CAD
            .strafePodX(0)   // Centered by design in CAD
            .distanceUnit(DistanceUnit.INCH)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);



    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
