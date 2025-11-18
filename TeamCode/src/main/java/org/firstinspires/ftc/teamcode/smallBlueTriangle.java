
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous
public class smallBlueTriangle extends LinearOpMode {
    private ShooterController shooter;
   // Declare our motors ----------------------------------------
    
    private DcMotor bl0, br1, fl2, fr3, cm4;
    private DcMotorEx launcher;
    private DcMotor intake, intake2;
    
    @Override
    public void runOpMode() {
        
        bl0 = hardwareMap.get(DcMotor.class, "bl0");
        br1 = hardwareMap.get(DcMotor.class, "br1");
        fl2 = hardwareMap.get(DcMotor.class, "fl2");
        fr3 = hardwareMap.get(DcMotor.class, "fr3");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        //cm4 = hardwareMap.get(DcMotor.class, "cm4");
        
        // Encoder logic -------------------------------------------------
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        // Set motor directions (adjust if movement is inverted) ----------
        fl2.setDirection(DcMotor.Direction.REVERSE);
        bl0.setDirection(DcMotor.Direction.REVERSE);
        fr3.setDirection(DcMotor.Direction.FORWARD);
        br1.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        
        // Set motor behavior ----------------------------------------------
        bl0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        
        
        
        
        
        shooter = new ShooterController(launcher, intake, intake2, this);
        

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        setDrive(1, 1, 1, 1, 1000);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()){
            shooter.update();
            
           

        }
    }
    
    public void setDrive(double frontLeft, double frontRight, double backLeft, double backRight, int position){
        // Reset Encoders
        fl2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        fr3.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        bl0.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        br1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        // Set target position
        fl2.setTargetPosition(position);
        fr3.setTargetPosition(position);
        bl0.setTargetPosition(position);
        br1.setTargetPosition(position);
        
        // Run position
        fl2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        
        // Apply power to motors
        fl2.setPower(frontLeft);
        fr3.setPower(-1*frontRight);
        bl0.setPower(-1*backLeft);
        br1.setPower(backRight);
        
        
            


        
        
    }
    
    
}
