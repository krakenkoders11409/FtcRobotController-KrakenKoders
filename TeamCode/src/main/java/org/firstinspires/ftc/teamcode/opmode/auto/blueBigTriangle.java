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

        Robot robot = new Robot(hardwareMap);

        //telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.drive.resetEncoders();
        robot.drive.setRunToPositionMode();
        robot.drive.setTargetForwardInches(-36, 0.4);

// DO NOT sleep here unless mechanically required
        robot.shooter.startShot(1, "short");

        while (opModeIsActive() && robot.shooter.isBusy()) {
            robot.shooter.update();
            robot.addTelemetry(telemetry);
            telemetry.update();
        }

// Shot is DONE here
        robot.drive.setTargetInches(0, 15, 0, -0.4);
        robot.shooter.liftBall();

// Keep subsystems alive
        while (opModeIsActive()) {
            robot.shooter.update();
        }

    }
}
