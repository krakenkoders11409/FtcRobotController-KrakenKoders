/*
Copyright 2025 FIRST Tech Challenge Team 11409

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;


@Autonomous
public class blueBigTriangle extends LinearOpMode {

    @Override
    public void runOpMode() {
        //telemetry.addData("Status", "Initialized");
        Robot robot = new Robot(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.limelight.periodic();

        while(opModeIsActive() && !robot.limelight.hasTarget()) {
            robot.limelight.periodic();
            telemetry.addLine("Searching for Tag...");
            telemetry.update();

            robot.drive.drive(0, 0, 0.15);

        robot.drive.drive(0, 0, 0);
        telemetry.addLine("Tag Found!");
        telemetry.update();

        while (opModeIsActive() && robot.limelight.hasTarget() && Math.abs(robot.limelight.getTx()) > 1.0) {
            robot.limelight.periodic();
            double turn = robot.limelight.getSteeringCorrections();
            robot.drive.drive(0, 0, turn);

            telemetry.addData("Tx = ", robot.limelight.getTx());
            telemetry.addData("Turn", turn);
            telemetry.update();
        }

        robot.drive.drive(0, 0, 0);

        robot.drive.resetEncoders();
        robot.drive.setRunToPositionMode();
        robot.drive.setTargetForwardInches(10, 0.8);

        robot.shooter.startShot(3, "short");

        while(opModeIsActive()) {
            robot.shooter.update();
        }
        }
    }
}
