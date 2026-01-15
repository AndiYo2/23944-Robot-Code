
package Autos;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;


@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }


    public static class Paths {
        public PathChain shootToFirst;
        public PathChain firstToShoot;
        public PathChain shootToSecond;
        public PathChain secondToGate1;
        public PathChain secondToGate2;
        public PathChain gateToShoot;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            shootToFirst = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.500, 8.500),
                                    new Pose(57.500, 30.000),
                                    new Pose(22.500, 38.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .setGlobalDeceleration()
                    .build();

            firstToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.500, 38.000),

                                    new Pose(56.500, 10.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .setGlobalDeceleration()
                    .build();

            shootToSecond = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.500, 10.500),
                                    new Pose(62.800, 60.000),
                                    new Pose(22.500, 62.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .setGlobalDeceleration()
                    .build();

            secondToGate1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(22.500, 62.000),
                                    new Pose(21.500, 65.250),
                                    new Pose(22.500, 70.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                    .setGlobalDeceleration()
                    .build();

            secondToGate2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.500, 70.500),

                                    new Pose(16.500, 70.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .setGlobalDeceleration()
                    .build();

            gateToShoot = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(16.500, 70.500),
                                    new Pose(49.745, 65.048),
                                    new Pose(57.500, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .setGlobalDeceleration()
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(57.500, 85.000),

                                    new Pose(22.500, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setGlobalDeceleration()
                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.500, 85.000),

                                    new Pose(57.500, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .setGlobalDeceleration()
                    .build();
        }
    }


    public int autonomousPathUpdate() {
        // Event markers will automatically trigger at their positions
        // Make sure to register NamedCommands in your RobotContainer
        return pathState;
    }


}
    