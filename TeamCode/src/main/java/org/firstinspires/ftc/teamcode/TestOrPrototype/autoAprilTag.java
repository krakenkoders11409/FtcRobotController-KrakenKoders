
package org.firstinspires.ftc.teamcode.TestOrPrototype;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;
import java.util.List;

//@Disabled 
@Autonomous

public class autoAprilTag extends LinearOpMode {
    
    private Robot robot;

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

        telemetry.addLine("Ready to drive!");
        telemetry.update();
        
        robot = new Robot(hardwareMap);

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
            robot.shooter.startShot(2, "short");
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
