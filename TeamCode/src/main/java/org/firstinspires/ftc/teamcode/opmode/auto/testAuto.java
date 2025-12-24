package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;


@Autonomous
public class testAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.drive.resetEncoders();
        robot.drive.setRunToPositionMode();
        robot.drive.setTargetForwardInches(-36, .8); // TODO: I'm not sure if a negative distance actually makes it go backwards.. it should right?

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
            while (opModeIsActive()) {
                robot.UpdatePeriodic(); // updates vision pose

                robot.UpdatePeriodic(); // updates vision pose

                if (robot.vision.hasFieldPose()) {
                    double x = robot.vision.getFieldX();
                    double y = robot.vision.getFieldY();
                    double yaw = robot.vision.getFieldYaw();

                    // auto drive logic here
                }
            }
            robot.shooter.update();
            robot.UpdatePeriodic();
        }
    }
}
