package pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.PathBuilder;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;



import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    static double yVelocity =64.26073113388904,
            xVelocity = 78.42103348379062,
            robotMass = 12.247;




    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(robotMass)
        .forwardZeroPowerAcceleration(-24.35247829)
        .lateralZeroPowerAcceleration(-58.772098)
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
    // PathConstraints(tValueConstraint, timeoutConstraint, brakingStrength, brakingStart)
    // brakingStart = how many inches from end to START braking (was 1.75, now 12 for earlier decel)
    // brakingStrength = how aggressively to brake (15 is strong)
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.05, 1);
    //bs 15

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

    /**
     * Simple passthrough to follower.pathBuilder() - V4.0 style (NO deceleration).
     * This is for testing without global deceleration.
     */
    public static PathBuilder pathBuilder(Follower follower) {
        return follower.pathBuilder();
    }
}
