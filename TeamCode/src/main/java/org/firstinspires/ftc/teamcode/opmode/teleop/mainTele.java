package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import org.firstinspires.ftc.teamcode.hardware.Robot;

//import org.firstinspires.ftc.teamcode.SimpleLedController;

// @Disabled
@TeleOp


public class mainTele extends LinearOpMode {
    private Robot robot;
    private boolean isRed = true;
    
    boolean APressedLast = false;
    boolean BPressedLast = false;
    boolean XPressedLast = false;
    boolean YPressedLast = false;

    boolean DPadLeftPressedLast = false;
    boolean DPadRightPressedLast = false;
    boolean DPadUpPressedLast = false;
    boolean DPadDownPressedLast = false;

    private String shotType = "";




    //@Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Ready to drive!");

        robot = new Robot(hardwareMap, telemetry);
        robot.shooter.angleUp();
        robot.shooter.angleDown();


        waitForStart();

        while (opModeIsActive()) {

            // Read joystick Values  for Gamepad 1 --------------------------------------
            double forward = -gamepad1.left_stick_y; // Forward is negative on the stick
            double strafe = gamepad1.left_stick_x;  // Strafe
            double turn = gamepad1.right_stick_x; // Rotation
            // Read Joystick Values for Gamepad 2 ---------------------------------------
            double horizontal = -gamepad2.right_stick_x;
            double vertical = -gamepad2.left_stick_y;

            // Change Speed --------------------------------------------------------------
            if (gamepad1.b && !BPressedLast) {
                robot.drive.toggleSlowMode();
            }



            // Turret ---------------------------------------------------------------
            // --- Aiming ---
            // Toggle AUTO AIM
            if (robot.turret.getState() == TurretSubsystem.State.AUTO) {
                robot.turret.disableAutoAim();
            } else {
                robot.turret.enableAutoAim();
            }

            if(gamepad2.dpad_left && !DPadLeftPressedLast) {
                robot.vision.clearAllowedTags();
                robot.vision.addAllowedTag(20);
            }
            if(gamepad2.dpad_right && !DPadRightPressedLast) {
                robot.vision.clearAllowedTags();
                robot.vision.addAllowedTag(24);
            }




            // --- Turret Control ---
            robot.turret.manualAiming(-gamepad2.right_stick_x);
            robot.turret.autoAim(robot.vision.getTx(), robot.vision.hasTarget());
            robot.turret.setTxOffset(0);
            robot.turret.update(-gamepad2.right_stick_x, robot.vision.getTx(), robot.vision.hasTarget());


            if (gamepad2.x && !XPressedLast) {
                robot.turret.toggleAutoAim();
            }
            XPressedLast = gamepad2.x;

            robot.shooter.manualAngling(gamepad2.left_stick_y);

            // --- Auto Shot Type ---
            if(robot.vision.getTagDistanceMeters() < 2.5 || !robot.vision.hasTarget()){
                shotType = "short";
            } else {
                shotType = "long";
            }
            if (gamepad2.b) {
                robot.shooter.startShot(1, shotType);
            }

            // --- Initiate a short shot ---
            if (gamepad2.y && !YPressedLast) {
                    robot.shooter.startShot(1, "short");
            }

            // --- Initiate a long shot ---
            if (gamepad2.a && !APressedLast) {
                    robot.shooter.startShot(1, "long");
            }

            // Manually control the intakes -----------------------------------------------
            // --- Intake ---
            if (!robot.shooter.isBusy()) {
                // Control the first intake
                if (gamepad2.left_bumper)  {
                    robot.shooter.startIntake(1);
                } else if (gamepad2.right_bumper ) {
                    robot.shooter.reverseIntake(0.5);
                //} else if (gamepad2.left_trigger > 0.3) {
//                    robot.shooter.unBlockIntake();
//                    robot.shooter.startIntake(1);
                } else {
                    robot.shooter.stopIntake();
                }

                // Control Outtake ----------------------------------------------
                if (gamepad2.right_trigger > 0) {
                    //telemetry.addLine("Velocity= " + launcher.getVelocity());
                    robot.shooter.startOuttake(1);
                } else if (gamepad2.left_trigger > 0.3) {
                    robot.shooter.startOuttake(-gamepad2.left_trigger);
                } else {
                    robot.shooter.stopOuttake();
                }
            }

            robot.drive.drive(forward, strafe, turn);

            // Loop updates
            robot.update();

            // Display info on driver station --------------------------------
            robot.addTelemetry(telemetry);
            telemetry.update();

        }
    }
}
