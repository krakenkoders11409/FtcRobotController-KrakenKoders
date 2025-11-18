package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class AprilTagReader {

    private AprilTagProcessor tagProcessor;
    private VisionPortal visionPortal;

    public AprilTagReader(HardwareMap hardwareMap) {
        // Make the AprilTag Processor
        tagProcessor = new AprilTagProcessor.Builder().build();

        // Build the Vision Portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(tagProcessor)
                .build();
    }

    public AprilTagProcessor getProcessor() {
        return tagProcessor;
    }

    public int getTagId() {
        if (tagProcessor.getDetections().size() > 0) {
            return tagProcessor.getDetections().get(0).id;
        }
        return -1; // nothing found
    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
