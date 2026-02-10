package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "BlueLong", group = "blue")
public class BlueLong extends LinearOpMode {
    private enum AutoStep {START_SHOTPOS, AIM, SHOOT, SHOT_PICKUP, ROTATETOPICKUP, PICKUP, BACK_UP, ROTATE_BACK, DONE }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);

        // --- Initialize vision + turret ---
        int primaryTargetTagID = 20;      // main target
        int anchorTag1 = 24;              // optional anchor
        //int anchorTag2 = 19;              // optional anchor
        double turretOffsetDeg = 2;       // turret physical offset

        robot.turret.initAim(turretOffsetDeg, primaryTargetTagID);
        robot.vision.addAllowedTag(anchorTag1);
        //robot.vision.addAllowedTag(anchorTag2);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for start
        waitForStart();

        // --- Setup robot ---
        robot.drive.resetEncoders();
        robot.shooter.angleUp();

        // Timer for sequential actions
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        AutoStep autoStep = AutoStep.START_SHOTPOS;
        final double AIM_TOLERANCE_DEG = 5.0;
        final double SHOOT_TIMEOUT_S = 4.0;
        boolean rotate = false;

        while (opModeIsActive() && autoStep != AutoStep.DONE) {

            // --- Update subsystems ---
            robot.vision.update();
            boolean hasTarget = robot.vision.hasTarget();
            double tx = robot.vision.getTx();
            robot.turret.update(0, tx, hasTarget);
            robot.shooter.update();
            robot.drive.update();

            // --- Telemetry ---
            telemetry.addData("Step", autoStep);
            telemetry.addData("HasTarget", hasTarget);
            telemetry.addData("TX", hasTarget ? String.format("%.2f", tx) : "N/A");
            telemetry.addData("TurretPos", robot.turret.turntableMotor.getCurrentPosition());
            telemetry.addData("TurretTXOffset", robot.turret.getTxOffset());
            telemetry.addData("ShooterBusy", robot.shooter.isBusy());
            telemetry.addData("DriveBusy", robot.drive.isBusy());
            telemetry.update();

            // --- Auto steps ---
            switch (autoStep) {

                case START_SHOTPOS:
                    robot.drive.resetEncoders();
                    robot.drive.setTargetDrive(2, 0, 0, 0.4);
                    robot.drive.setRunToPositionMode();
                    timer.reset();
                    autoStep = AutoStep.AIM;
                    break;

                case AIM:
                    if (hasTarget) {
                        double adjusted = tx - robot.turret.getTxOffset();
                        if (Math.abs(adjusted) < AIM_TOLERANCE_DEG && timer.seconds() > 1) {
                            robot.shooter.startShot(1, "autoLong");
                            timer.reset();
                            autoStep = AutoStep.SHOOT;
                        }
                    }
                    break;

                case SHOOT:
                    if (!robot.shooter.isBusy() || timer.seconds() > SHOOT_TIMEOUT_S) {
                        robot.drive.resetEncoders();
                        robot.drive.setTargetDrive(24, 0, 0, 0.8);
                        robot.drive.setRunToPositionMode();
                        timer.reset();
                        autoStep = AutoStep.SHOT_PICKUP;
                    }
                    break;

                case SHOT_PICKUP:
                    if (timer.seconds() > 4) {
                        robot.drive.setTargetDrive(0, 0, -90, 0.5);
                        robot.drive.setRunToPositionMode();
                        rotate = true;
                        timer.reset();
                        autoStep = AutoStep.ROTATETOPICKUP;
                    }
                    break;

                case ROTATETOPICKUP:
                    if (timer.seconds() > 2) {
                        robot.shooter.startIntake(1);
                        robot.shooter.startOuttake(-0.5);
                        robot.drive.setTargetDrive(45, 0, 0, 0.5);
                        robot.drive.setRunToPositionMode();
                        timer.reset();
                        autoStep = AutoStep.PICKUP;
                    }
                    break;

                case PICKUP:
                    if (timer.seconds() > 7) {
                        robot.shooter.stopIntake();
                        robot.shooter.stopOuttake();
                        robot.drive.setTargetDrive(-40, 0, 0, 0.5);
                        robot.drive.setRunToPositionMode();
                        timer.reset();
                        autoStep = AutoStep.BACK_UP;
                    }
                    break;

                case BACK_UP:
                    if (timer.seconds() > 1) {
                        robot.drive.setTargetDrive(0, 0, 70, -0.5);
                        timer.reset();
                        if (timer.seconds() > 3) {
                            robot.drive.setTargetDrive(-20, 0, 0, 0.5);
                            robot.shooter.startShot(1, "autoLong");
                            timer.reset();
                        }
                    }
                    if (!robot.drive.isBusy()) autoStep = AutoStep.DONE;
                    break;

                case DONE:
                    // optionally stop motors
                    robot.drive.stop();
                    break;
            }

            idle();
        }

        telemetry.addData("Auto", "Finished");
        telemetry.update();
    }
}
