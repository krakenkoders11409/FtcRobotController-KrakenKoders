
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.List;

//@Disabled 
@Autonomous

public class autoAprilTag extends LinearOpMode {
    
    private ShooterController shooter;
    
    //Declare Motors
    private DcMotor bl0, br1, fl2, fr3, cm4;
    private DcMotorEx launcher;
    private DcMotor intake, intake2;
    





    @Override
    public void runOpMode() {
        
        // April Tag Detection ------------------------------------
        
        // Make the list of ball pattern
        List<String> pattern1 = new ArrayList<>();
        pattern1.add("Green");
        pattern1.add("Purple");
        
        
         // Create the AprilTag reader object
        AprilTagReader tagReader = new AprilTagReader(hardwareMap);

        int detectedTag = -1;

        // Read tags during INIT
        while (opModeInInit()) {
            detectedTag = tagReader.getTagId();
            telemetry.addData("Detected Tag", detectedTag);
            telemetry.update();
        }
        
        // Map Hardware -------------------------------------------
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
        
        //cm4.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Ready to drive!");
        telemetry.update();
        
        // Shooter -----------------------------------------------------------
        shooter = new ShooterController(launcher, intake, intake2, this);
        
        

        waitForStart();
        
        
        
        
         // Use detected tag inside autonomous
        if (detectedTag == 1) {
            // left
        } else if (detectedTag == 2) {
            // middle
        } else if (detectedTag == 3) {
            // right
        }
        
        if (detectedTag == 21) { //PPG
            shooter.startShot(1, "short");
            sleep(100);
            intake.setPower(1);
            launcher.setPower(.5);
            sleep(100);
            shooter.startShot(1, "short");
        }
        // When done
        tagReader.stop();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Detected Tag", detectedTag);
            telemetry.update();

        }
    }
}
