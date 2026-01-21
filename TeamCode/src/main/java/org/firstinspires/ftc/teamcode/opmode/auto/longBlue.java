package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "longBlue", group = "long")
public class longBlue extends LinearOpMode {
    private enum AutoStep { POSITION, AIM, SHOOT, DRIVE, ROTATE, GRAB, BACK_UP, ROTATE_BACK, DONE }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // --- Set up wheels ---
        robot.drive.resetEncoders();

        // --- Set up hood ---
        robot.shooter.angleUp();
        robot.shooter.angleDown();

        robot.shooter.angleUp();
        robot.shooter.angleUp();
        robot.shooter.angleUp();
        robot.shooter.angleUp();


        // --- Set up vision / turret ---
        // Set turret TX offset to +10 degrees <- Left (adjust if your sign convention differs)
        robot.turret.setTxOffset(2);
        robot.turret.setTurretKP(0.03);

        // Prefer explicit enable if available. If not, toggle is a fallback.
        try {
            // if API has enableAutoAim(boolean)
            robot.turret.getClass().getMethod("enableAutoAim", boolean.class).invoke(robot.turret, true);
        } catch (Exception e) {
            // fallback to toggle if explicit method not present
            robot.turret.toggleAutoAim();
        }

        robot.vision.clearAllowedTags();
        robot.vision.addAllowedTag(20);

        // Timer for sequential actions and timeouts
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // --- IMPORTANT: initialize autoStep ---
        AutoStep autoStep = AutoStep.POSITION;

        // safety timeout values (adjust as needed)
        final double SHOOT_TIMEOUT_S = 4;    // max seconds to wait for shooter
        final double AIM_TOLERANCE_DEG = 5.0;  // aim tolerance in degrees
        boolean rotate = false;

        while (opModeIsActive() && autoStep != AutoStep.DONE) {
            // Update subsystems
            robot.vision.update();
            double tx = robot.vision.getTx();            // may be undefined if no target
            boolean hasTarget = robot.vision.hasTarget();

            // Auto-aim turret - pass hasTarget to avoid NaNs inside turret update
            robot.turret.update(0, tx, hasTarget);

            // Shooter update
            robot.shooter.update();

            // Drive update
            robot.drive.update();

            // Telemetry for debugging
            telemetry.addData("Step", autoStep);
            telemetry.addData("HasTarget", hasTarget);
            telemetry.addData("Tx", hasTarget ? String.format("%.2f", tx) : "N/A");
            telemetry.addData("TxOffset", robot.turret.getTxOffset());
            telemetry.addData("ShooterBusy", robot.shooter.isBusy());
            telemetry.addData("DriveBusy", robot.drive.isBusy());
            telemetry.addData("Outtake Velocity", robot.shooter.getOuttakeVelocity());
            telemetry.addData("Rotate", rotate);


            switch (autoStep) {
                case POSITION:
                    robot.drive.resetEncoders();
                    robot.drive.setTargetDrive(2, 0, 0, 0.4);
                    robot.drive.setRunToPositionMode();
                    timer.reset();
                    autoStep = AutoStep.AIM;


                case AIM:
                    // Only consider aiming if we have a target
                    if (hasTarget) {
                        double adjusted = tx - robot.turret.getTxOffset(); // target minus offset
                        telemetry.addData("AdjustedTx", String.format("%.2f", adjusted));
                        if (Math.abs(adjusted) < AIM_TOLERANCE_DEG) {
                            if (timer.seconds() > 1) {
                                // Start shooting sequence
                                autoStep = AutoStep.SHOOT;
                                robot.shooter.startShot(1, "autoLong");
                                timer.reset(); // begin shooter timeout
                            }
                        }
                    } else {
                        // Optionally: implement a search routine (rotate turret) if no target
                        telemetry.addData("Aim", "No target - searching (optional)");
                    }
                    break;

                case SHOOT:
                    // Proceed when shooter finished OR when timeout elapsed
                    if (!robot.shooter.isBusy()) {
                        // advance to driving
                        timer.reset();
                        if (timer.seconds() > 2) {
                            autoStep = AutoStep.DRIVE;
                            robot.drive.resetEncoders();
                            robot.drive.setTargetDrive(24, 0, 0, 0.8);
                            robot.drive.setRunToPositionMode();
                            autoStep = AutoStep.DRIVE;
                        }

                    } else if (timer.seconds() > SHOOT_TIMEOUT_S) {
                        // shooter stuck — bail out to next step to avoid stall
                        telemetry.addData("WARN", "Shooter timeout, continuing.");
                        robot.drive.resetEncoders();
                        robot.drive.setTargetDrive(24, 0, 0, 0.8);
                        robot.drive.setRunToPositionMode();
                        timer.reset();
                        autoStep = AutoStep.DRIVE;

                    }
                    break;

                case DRIVE:
                    if (timer.seconds() > 4) {
                        telemetry.addLine("Rotation");
                        robot.drive.setTargetDrive(0, 0, -90, 0.5);
                        rotate = true;
                        robot.drive.setRunToPositionMode();
                        autoStep = AutoStep.ROTATE;
                        timer.reset();
                        }
                    break;

                case ROTATE:
                    if(timer.seconds() > 2) {
                        robot.shooter.startIntake(1);
                        robot.shooter.startOuttake(-0.5);
                        robot.drive.setTargetDrive(45, 0, 0, 0.07);
                        robot.drive.setRunToPositionMode();
                        autoStep = AutoStep.GRAB;
                        timer.reset();
                    }
                    break;

                case GRAB:
                    if(timer.seconds() > 7) {
                        robot.shooter.stopIntake();
                        robot.shooter.stopOuttake();
                        robot.drive.setTargetDrive(-40, 0, 0, 0.5);
                        robot.drive.setRunToPositionMode();
                        timer.reset();
                        autoStep = AutoStep.BACK_UP;
                        }
                    break;

                case BACK_UP:

                    if(timer.seconds() > 1) {
                        robot.drive.setTargetDrive(0, 0, 70, -0.5);
                        timer.reset();
                        if (timer.seconds() > 3) {
                            robot.drive.setTargetDrive(-20, 0, 0, 0.5);
                            robot.shooter.startShot(1, "autoLong");
                            autoStep = AutoStep.BACK_UP;
                            timer.reset();
                        }
                    }


                    if (!robot.drive.isBusy()) {
                        autoStep = AutoStep.DONE;
                    }
                    break;





                case DONE:
                    // Optionally stop motors or set a safe state
                    break;
            }

            telemetry.update();
            // allow other threads / hardware to run
            idle();
        }

        // cleanup / final telemetry
        telemetry.addData("Auto", "Finished");
        telemetry.update();
    }
}
