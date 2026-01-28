package org.firstinspires.ftc.teamcode.opmode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Example Pedro Pathing autonomous for Blue alliance.
 *
 * IMPORTANT: This is a TEMPLATE. Before this will work accurately, you must:
 * 1. Configure your two-wheel localizer in Constants.java
 * 2. Run the Tuning OpModes to calibrate your robot
 * 3. Adjust the coordinates below to match your field positions
 *
 * Coordinate System:
 * - X increases going forward (toward the submersible/net zone)
 * - Y increases going left
 * - Heading is in radians (0 = facing forward, Math.PI/2 = facing left)
 */
@Autonomous(name = "Pedro Blue Auto (EXAMPLE)", group = "Pedro")
@Disabled  // Remove this line once you've tuned your robot
public class PedroBlueAuto extends LinearOpMode {

    // State machine for autonomous sequence
    private enum AutoState {
        DRIVE_TO_SHOOT,
        AIM_AND_SHOOT,
        DRIVE_TO_PICKUP,
        GRAB_BALLS,
        DRIVE_BACK,
        SHOOT_AGAIN,
        DONE
    }

    // Pedro Pathing follower
    private Follower follower;

    // Your existing robot subsystems
    private Robot robot;

    // Path definitions - these are created in buildPaths()
    private PathChain driveToShootPath;
    private PathChain driveToPickupPath;
    private PathChain driveBackPath;

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing follower
        follower = Constants.createFollower(hardwareMap);

        // Initialize your existing robot subsystems
        robot = new Robot(hardwareMap, telemetry);

        // Define the starting position on the field (adjust for your setup)
        // This is where the robot starts when you press INIT
        Pose startPose = new Pose(0, 0, Math.toRadians(0));  // x, y, heading in radians
        follower.setStartingPose(startPose);

        // Build all the paths during init (before the match starts)
        buildPaths();

        // Setup subsystems (same as your current auto)
        robot.shooter.angleUp();
        robot.turret.setTxOffset(2);
        robot.turret.setTurretKP(0.03);
        robot.vision.clearAllowedTags();
        robot.vision.addAllowedTag(20);

        telemetry.addData("Status", "Initialized - Pedro Pathing Ready");
        telemetry.addData("Start Pose", startPose.toString());
        telemetry.update();

        waitForStart();

        // Start following the first path
        AutoState state = AutoState.DRIVE_TO_SHOOT;
        ElapsedTime timer = new ElapsedTime();
        follower.followPath(driveToShootPath, true);  // true = hold end position

        while (opModeIsActive() && state != AutoState.DONE) {
            // CRITICAL: Update follower every loop - this is what makes the robot move!
            follower.update();

            // Update your existing subsystems
            robot.vision.update();
            robot.shooter.update();
            double tx = robot.vision.getTx();
            boolean hasTarget = robot.vision.hasTarget();
            robot.turret.update(0, tx, hasTarget);

            // Telemetry
            telemetry.addData("State", state);
            telemetry.addData("Pose", follower.getPose().toString());
            telemetry.addData("Path Busy", follower.isBusy());

            switch (state) {
                case DRIVE_TO_SHOOT:
                    // Wait for path to complete
                    if (!follower.isBusy()) {
                        state = AutoState.AIM_AND_SHOOT;
                        timer.reset();
                    }
                    break;

                case AIM_AND_SHOOT:
                    // Similar to your current AIM logic
                    if (hasTarget && Math.abs(tx - robot.turret.getTxOffset()) < 5.0) {
                        if (timer.seconds() > 0.5) {
                            robot.shooter.startShot(1, "autoLong");
                            timer.reset();
                        }
                    }

                    // Move on after shooting or timeout
                    if (!robot.shooter.isBusy() && timer.seconds() > 1.0) {
                        follower.followPath(driveToPickupPath, true);
                        state = AutoState.DRIVE_TO_PICKUP;
                        timer.reset();
                    } else if (timer.seconds() > 4.0) {
                        // Timeout - move on anyway
                        follower.followPath(driveToPickupPath, true);
                        state = AutoState.DRIVE_TO_PICKUP;
                        timer.reset();
                    }
                    break;

                case DRIVE_TO_PICKUP:
                    if (!follower.isBusy()) {
                        robot.shooter.startIntake(1);
                        robot.shooter.startOuttake(-0.5);
                        state = AutoState.GRAB_BALLS;
                        timer.reset();
                    }
                    break;

                case GRAB_BALLS:
                    // Intake for a set time
                    if (timer.seconds() > 3.0) {
                        robot.shooter.stopIntake();
                        robot.shooter.stopOuttake();
                        follower.followPath(driveBackPath, true);
                        state = AutoState.DRIVE_BACK;
                        timer.reset();
                    }
                    break;

                case DRIVE_BACK:
                    if (!follower.isBusy()) {
                        state = AutoState.SHOOT_AGAIN;
                        timer.reset();
                    }
                    break;

                case SHOOT_AGAIN:
                    if (hasTarget && Math.abs(tx - robot.turret.getTxOffset()) < 5.0) {
                        robot.shooter.startShot(3, "autoLong");
                    }

                    if (!robot.shooter.isBusy() && timer.seconds() > 1.0) {
                        state = AutoState.DONE;
                    } else if (timer.seconds() > 5.0) {
                        state = AutoState.DONE;
                    }
                    break;

                case DONE:
                    break;
            }

            telemetry.update();
        }

        telemetry.addData("Auto", "Complete");
        telemetry.update();
    }

    /**
     * Build all paths during initialization.
     *
     * Path types:
     * - BezierLine: Straight line from A to B
     * - BezierCurve: Curved path using control points
     *
     * All coordinates are in INCHES from your starting position.
     */
    private void buildPaths() {
        // Path 1: Drive forward to shooting position
        // BezierLine creates a straight path
        driveToShootPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(0, 0, 0),      // Start (current position)
                        new Pose(24, 0, 0)      // End (24 inches forward)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))  // Keep heading constant
                .build();

        // Path 2: Drive to ball pickup area with a curve
        // BezierCurve allows curved paths using control points
        driveToPickupPath = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(24, 0, 0),     // Start
                        new Pose(36, 12, 0),    // Control point (shapes the curve)
                        new Pose(48, 24, 0)     // End
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))  // Rotate during path
                .build();

        // Path 3: Drive back to shooting position
        driveBackPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(48, 24, 0),    // Start (at pickup)
                        new Pose(24, 0, 0)      // End (back at shoot position)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                .build();
    }
}
