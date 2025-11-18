package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;


@Disabled
@TeleOp


public class OmniDrive extends LinearOpMode {
    // Declare our motors ----------------------------------------
    
    private DcMotor bl0, br1, fl2, fr3, cm4;
    private DcMotor launcher;
    
    //Select Speed -------------------------------------------------
    
    double selectSpeed = 1.0;
    boolean rightAPressedLast = false;
    boolean rightBPressedLast = false;
    boolean rightXPressedLast = false;
    boolean rightYPressedLast = false;
    
    
    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors from hardware map ------------------------
        
        bl0 = hardwareMap.get(DcMotor.class, "bl0");
        br1 = hardwareMap.get(DcMotor.class, "br1");
        fl2 = hardwareMap.get(DcMotor.class, "fl2");
        fr3 = hardwareMap.get(DcMotor.class, "fr3");
        launcher = hardwareMap.get(DcMotor.class, "arm");
        //cm4 = hardwareMap.get(DcMotor.class, "cm4");
        
        // Encoder logic -------------------------------------------------
        
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        // Set motor directions (adjust if movement is inverted) ----------
        
        fl2.setDirection(DcMotor.Direction.REVERSE);
        bl0.setDirection(DcMotor.Direction.REVERSE);
        fr3.setDirection(DcMotor.Direction.FORWARD);
        br1.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        
        // Set motor behavior ----------------------------------------------
        
        bl0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //cm4.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Ready to drive!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Read joystick values ------------------------------------
            
            double y = -gamepad1.left_stick_y; // Forward is negative on the stick
            double x = gamepad1.left_stick_x;  // Strafe
            double rx = gamepad1.right_stick_x; // Rotation
            
            // Optional: Slow mode ----------------------------------------
            double speedMultiplier = gamepad1.right_bumper ? 0.4 : 1.0;

            // Calculate motor powers (mecanum math) -----------------------
            double flPower = (y + x + rx) * speedMultiplier;
            double blPower = (y - x + rx) * speedMultiplier;
            double frPower = (y - x - rx) * speedMultiplier;
            double brPower = (y + x - rx) * speedMultiplier;
            
            
            // Normalize powers if any exceed 1.0 ---------------------------
            
            double max = Math.max(1.0, Math.max(Math.abs(flPower),
                          Math.max(Math.abs(blPower), Math.max(Math.abs(frPower), Math.abs(brPower)))));
            flPower /= max;
            blPower /= max;
            frPower /= max;
            brPower /= max;
            
            //Change Speed ---------------------------------------------------
            
            if (gamepad1.a && !rightBPressedLast) {
                if (selectSpeed == 1.0){
                    selectSpeed = 0.5;
                
                } else {
                   selectSpeed = 1.0; 
                }
            }
            telemetry.addData("Speed Mode", selectSpeed == 1.0 ? "Fast" : "Slow");
            
            //Launcher Stuff :) -----------------------------------------
            
            if (gamepad1.x && !rightXPressedLast) {
                launcher.setPower(1);
                
            } else {
                launcher.setPower(0);
            }
            
            
            // Speed selcetion --------------------------------------------
            
            fl2.setPower(flPower*selectSpeed);
            bl0.setPower(blPower*selectSpeed);
            fr3.setPower(frPower*selectSpeed);
            br1.setPower(brPower*selectSpeed);
        

            // Display info on driver station -----------------------------
            
            telemetry.addData("FL Power", flPower);
            telemetry.addData("FR Power", (-1 *frPower));
            telemetry.addData("BL Power", blPower);
            telemetry.addData("BR Power", brPower);
            telemetry.update();
        }
    }
}
