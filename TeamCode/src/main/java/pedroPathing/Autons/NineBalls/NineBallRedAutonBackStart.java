//package pedroPathing.Autons.NineBalls;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import pedroPathing.Constants;
//import teleOps.OpModeTemplate;
//
//
//@Autonomous(name = "BackRedAuton")
//public class NineBallRedAutonBackStart extends OpModeTemplate {
//    private Follower follower;
//    private Timer pathTimer, actionTimer, opmodeTimer;
//    private int pathState;
//
//    private Path startToShoot;
//
//    private PathChain
//            shootToFirst,
//            firstToShoot,
//            shootToSecond,
//            secondToShoot,
//            shootToStop;
//
//    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));
//    private final Pose endPose = new Pose(116, 72, Math.toRadians(0));
//
//    private final Pose shootPose = new Pose(90,90, Math.toRadians(45));
//    private final Pose secondaryShootPose = new Pose(90,90, Math.toRadians(40));
//    private final Pose firstPickupPose = new Pose(120,83.5, Math.toRadians(0));
//    private final Pose secondPickupPose = new Pose(120,55.5, Math.toRadians(0));
//
//    private final Pose shootToFirstControlPoint = new Pose(85.5,74);
//    private final Pose shootToSecondControlPoint = new Pose(82,55);
//
//
//    public void buildPaths(){
//        startToShoot = new Path(new BezierLine(startPose, shootPose));
//        startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());
//
//        shootToFirst = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, shootToFirstControlPoint, firstPickupPose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), firstPickupPose.getHeading())
//                .build();
//        firstToShoot = follower.pathBuilder()
//                .addPath(new BezierLine(firstPickupPose,secondaryShootPose))
//                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), secondaryShootPose.getHeading())
//                .build();
//        shootToSecond = follower.pathBuilder()
//                .addPath(new BezierCurve(secondaryShootPose, shootToSecondControlPoint, secondPickupPose))
//                .setLinearHeadingInterpolation(secondaryShootPose.getHeading(), secondPickupPose.getHeading())
//                .build();
//        secondToShoot = follower.pathBuilder()
//                .addPath(new BezierLine(secondPickupPose, shootPose))
//                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
//                .build();
//        shootToStop = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, endPose))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
//                .build();
//    }
//
//    public void setPathState(int i){
//        pathState = i;
//        pathTimer.resetTimer();
//    }
//
//
//    public void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                if (follower.isBusy())
//                    break;
//                follower.followPath(startToShoot, true);
//                setPathState(1);
//
//            case 1:
//                if (follower.isBusy())
//                    break;
//
//
//                follower.followPath(shootToFirst, true);
//                follower.setMaxPower(.5);
//                setPathState(2);
//
//            case 2:
//                if (follower.isBusy())
//                    break;
//                follower.followPath(firstToShoot, true);
//
//                follower.setMaxPower(1);
//                setPathState(3);
//
//            case 3:
//                if (follower.isBusy()){
//                    break;
//                }
//
//                follower.followPath(shootToSecond,true);
//                setPathState(4);
//                follower.setMaxPower(.5);
//
//            case 4:
//                if(follower.isBusy()) {
//                    break;
//                }
//                follower.followPath(secondToShoot,true);
//
//                follower.setMaxPower(1);
//                setPathState(5);
//
//            case 5:
//                if(follower.isBusy())
//                    break;
//
//
//                follower.followPath(shootToStop, true);
//                setPathState(6);
//        }
//    }
//    /** This method is called once at the init of the OpMode. **/
//
//    @Override
//    public void initialize() {
//
//    }
//}