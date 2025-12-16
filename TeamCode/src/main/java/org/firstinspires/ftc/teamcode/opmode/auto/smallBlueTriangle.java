
package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;


@Autonomous
public class smallBlueTriangle extends LinearOpMode {
    
    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);
        

        //telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.drive.resetEncoders();
        robot.drive.setRunToPositionMode();
        robot.drive.setTargetInches(24, 10, 0, 0.4);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
            robot.shooter.update();
        }
    }
}
