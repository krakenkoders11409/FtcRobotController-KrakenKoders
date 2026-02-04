package org.firstinspires.ftc.teamcode.opmode.auto.Pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@Autonomous
public class PedroBlueShort extends OpMode {

    Robot robot;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState{
        //START_POSITION -> END_POSITION
        //DRIVE > MOVEMENT STATE
        // SHOOT > SCORE ARTIFACTS

        // DRIVE_STARTPOS_SHOOTPOS
        DRIVE_STARTPOS_SHOOTPOS,
        SHOOTPRELOAD,
        DRIVE_SHOOTPOS_PICKUPPOS,
        DRIVE_PICKUPPOS_SHOOTPOS,
        SHOOTPICKUP,
        DONE
    }

    PathState pathState;
    private final Pose startPose = new Pose(21.502318392581138, 121.91035548686244, Math.toRadians(135));
    private final Pose shootPose = new Pose(54.30602782071097, 88.97063369397219, Math.toRadians(135));
    private final Pose pickupPose = new Pose(17.748068006182372, 83.62287480680061, Math.toRadians(180));


    private PathChain driveStartPosShootPos;
    private PathChain driveShootPosPickupPos;
    private PathChain drivePickupPosShootPos;



    public void buildPaths() {
        // START CORD > END CORDS
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosPickupPos = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, pickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), pickupPose.getHeading())
                .build();
        drivePickupPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(pickupPose, shootPose))
                .setLinearHeadingInterpolation(pickupPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOTPOS:
                follower.followPath(driveStartPosShootPos, true);
                pathState = pathState.SHOOTPRELOAD;
                break;

            case SHOOTPRELOAD:
                if (!follower.isBusy()) {
                    robot.shooter.startShot(3, "short");
                    pathState = pathState.DRIVE_SHOOTPOS_PICKUPPOS;
                }
                break;

            case DRIVE_SHOOTPOS_PICKUPPOS:
               if (!robot.shooter.isBusy()) {
                   follower.followPath(driveShootPosPickupPos, true);
                   pathState = PathState.DRIVE_PICKUPPOS_SHOOTPOS;
               }
                break;

               case DRIVE_PICKUPPOS_SHOOTPOS:
                   if (!follower.isBusy()){
                       follower.followPath(drivePickupPosShootPos, true);
                       pathState = PathState.SHOOTPICKUP;
                   }
                   break;

            case SHOOTPICKUP:
                if (!follower.isBusy()) {
                    robot.shooter.startShot(3, "short");
                    pathState = PathState.DONE;
                }

            default:
                telemetry.addLine("PathState not found");
        }
    }

        public void setPathState(PathState newState) {
            pathState = newState;
            pathTimer.resetTimer();
        }


    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        pathState = PathState.DRIVE_STARTPOS_SHOOTPOS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

    }


    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        robot.turret.initAim(0, 20);
    }
    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        robot.turret.update(0, robot.vision.getTx(), robot.vision.hasTarget());
        robot.update();

        telemetry.addData("Path State: ", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Path Timer", pathTimer.getElapsedTime());
        telemetry.addData("OpMode Timer", opModeTimer.getElapsedTime());
    }
}
