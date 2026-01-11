package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import org.firstinspires.ftc.teamcode.hardware.Robot;

//import org.firstinspires.ftc.teamcode.SimpleLedController;

// @Disabled
@TeleOp


public class Tuning extends LinearOpMode {
    private Robot robot;
    private boolean isRed = true;

    boolean APressedLast = false;
    boolean BPressedLast = false;
    boolean XPressedLast = false;
    boolean YPressedLast = false;

    // Flywheel tuning
    private double flywheelVelocity = 1800;   // starting velocity (ticks/sec)
    private static final double VELOCITY_STEP = 50;
    private static final double MAX_VELOCITY = 2600;
    private static final double MIN_VELOCITY = 0;

    // Button state tracking
    private boolean dpadUpLast = false;
    private boolean dpadDownLast = false;


    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Ready to drive!");

        robot = new Robot(hardwareMap, telemetry);
        robot.shooter.angleUp();
        robot.shooter.angleDown();


        waitForStart();

        while (opModeIsActive()) {
            robot.update();

            double forward = -gamepad1.left_stick_y; // Forward is negative on the stick
            double strafe = gamepad1.left_stick_x;  // Strafe
            double turn = gamepad1.right_stick_x; // Rotation
            // Read Joystick Values for Gamepad 2 ---------------------------------------
            double horizontal = -gamepad2.right_stick_x;
            double vertical = -gamepad2.left_stick_y;

            robot.shooter.manualAngling(gamepad2.left_stick_y);

            boolean dpadUp = gamepad2.dpad_up;
            boolean dpadDown = gamepad2.dpad_down;

// Increase speed
            if (dpadUp && !dpadUpLast) {
                flywheelVelocity += VELOCITY_STEP;
            }

// Decrease speed
            if (dpadDown && !dpadDownLast) {
                flywheelVelocity -= VELOCITY_STEP;
            }

// Clamp velocity
            flywheelVelocity = Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, flywheelVelocity));

// Apply to motor
            robot.shooter.startOuttake(flywheelVelocity);

// Save last states
            dpadUpLast = dpadUp;
            dpadDownLast = dpadDown;

            if (!robot.shooter.isBusy()) {
                // Control the first intake
                if (gamepad2.left_bumper) {
                    robot.shooter.startIntake(1);
                } else if (gamepad2.right_bumper) {
                    robot.shooter.reverseIntake(0.5);
                } else if (gamepad2.left_trigger > 0.3) {
                    robot.shooter.unBlockIntake();
                    robot.shooter.startIntake(1);

                } else {
                    robot.shooter.stopIntake();
                }
                robot.drive.drive(forward, strafe, turn);
// Telemetry
                telemetry.addData("Flywheel Velocity", flywheelVelocity);
                robot.addTelemetry(telemetry);
                telemetry.update();


            }
        }
    }
}

