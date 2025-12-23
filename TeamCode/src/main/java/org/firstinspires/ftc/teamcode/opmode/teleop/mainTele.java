package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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


    //@Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Ready to drive!");

        robot = new Robot(hardwareMap, telemetry);


        waitForStart();

        while (opModeIsActive()) {
            robot.UpdatePeriodic();

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

            // Lift Ball -----------------------------------------------------------------
            if (gamepad2.x && !XPressedLast) {
                robot.shooter.liftBall();
            }

            // Turn Turret ---------------------------------------------------------------
            robot.turret.manualAiming(horizontal);
            // Change Launch Angle -------------------------------------------------------
            robot.shooter.manualAngling(vertical);
            // Initiate a short shot -----------------------------------------------------
            if (gamepad2.y && !YPressedLast) {
                    robot.shooter.startShot(1, "short");
            }

            // Initiate a long shot ------------------------------------------------------
            if (gamepad2.a && !APressedLast) {
                    robot.shooter.startShot(1, "long");
            }
            // Manually control the intakes -----------------------------------------------
            if (!robot.shooter.isBusy()) {
                // Control the first intake
                if (gamepad2.left_bumper)  {
                    robot.shooter.startIntake();
                } else {
                    robot.shooter.stopIntake();
                }
                if (gamepad2.dpad_up) {
                    robot.shooter.liftBall();
                }
                
                // Control the second intake ----------------------------------------------
                if (gamepad2.right_trigger > 0) {
                    //telemetry.addLine("Velocity= " + launcher.getVelocity());
                    robot.shooter.startOuttake();
                } else {
                    robot.shooter.stopOuttake();
                }
            }

            robot.drive.drive(forward, strafe, turn);

            // Loop updates
            robot.UpdatePeriodic();

            // Display info on driver station --------------------------------
            robot.addTelemetry(telemetry);
            telemetry.update();
        }
    }
} 
