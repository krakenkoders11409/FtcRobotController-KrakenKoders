package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous

public class leftSideShove extends LinearOpMode {

private DcMotor bl0, br1, fl2, fr3, cm4;

    @Override
    public void runOpMode() {
        // Name motors 
        bl0 = hardwareMap.get(DcMotor.class, "bl0");
        br1 = hardwareMap.get(DcMotor.class, "br1");
        fl2 = hardwareMap.get(DcMotor.class, "fl2");
        fr3 = hardwareMap.get(DcMotor.class, "fr3");
        
        // Set motor directions (adjust if movement is inverted)
        fl2.setDirection(DcMotor.Direction.REVERSE);
        bl0.setDirection(DcMotor.Direction.REVERSE);
        fr3.setDirection(DcMotor.Direction.FORWARD);
        br1.setDirection(DcMotor.Direction.FORWARD);
        // Set motor behavior
        bl0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        // Chess knight 
        bl0.setPower(1);
        br1.setPower(1);
        fl2.setPower(1);
        fr3.setPower(1);
        sleep(500);
        // Stop
        bl0.setPower(0);
        br1.setPower(0);
        fl2.setPower(0);
        fr3.setPower(0);
        sleep(100);
        
        /*
        // Left
        bl0.setPower(-0.5);
        br1.setPower(0.5);
        fl2.setPower(0.5);
        fr3.setPower(-0.5);
        sleep(1000);
        // Stop
        bl0.setPower(0);
        br1.setPower(0);
        fl2.setPower(0);
        fr3.setPower(0);
        */
        
        

    }
}
