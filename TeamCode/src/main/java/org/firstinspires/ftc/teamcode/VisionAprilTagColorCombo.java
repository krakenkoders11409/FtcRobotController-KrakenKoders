package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;
@Disabled
@TeleOp(name = "Concept: AprilTag + Color Sensor", group = "Concept")
public class VisionAprilTagColorCombo extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private PredominantColorProcessor colorSensor;

    @Override
    public void runOpMode() {

        // Initialize both processors
        initProcessors();

        telemetry.addLine("DS preview on/off: 3 dots > Camera Stream");
        telemetry.addLine("Press START to begin");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // --- AprilTag telemetry ---
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addLine("=== AprilTag Detections ===");
            telemetry.addData("# Detected", detections.size());

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("ID %d: %s", detection.id, detection.metadata.name));
                    telemetry.addLine(String.format("XYZ (in)  %.1f, %.1f, %.1f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY (deg) %.1f, %.1f, %.1f", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                } else {
                    telemetry.addLine(String.format("ID %d: Unknown", detection.id));
                }
            }

            // --- Color sensor telemetry ---
            telemetry.addLine("\n=== Color Sensor ===");
            PredominantColorProcessor.Result colorResult = colorSensor.getAnalysis();
            telemetry.addData("Best Match", colorResult.closestSwatch);
            telemetry.addLine(String.format("RGB   (%3d, %3d, %3d)",
                    colorResult.RGB[0], colorResult.RGB[1], colorResult.RGB[2]));
            telemetry.addLine(String.format("HSV   (%3d, %3d, %3d)",
                    colorResult.HSV[0], colorResult.HSV[1], colorResult.HSV[2]));
            telemetry.addLine(String.format("YCrCb (%3d, %3d, %3d)",
                    colorResult.YCrCb[0], colorResult.YCrCb[1], colorResult.YCrCb[2]));

            telemetry.update();

            // Camera stream control
            if (gamepad1.dpad_down) visionPortal.stopStreaming();
            if (gamepad1.dpad_up) visionPortal.resumeStreaming();

            sleep(20);
        }

        visionPortal.close();
    }

    private void initProcessors() {
        // AprilTag processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Color processor
        colorSensor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.BLACK,
                        PredominantColorProcessor.Swatch.WHITE)
                .build();

        // VisionPortal combines both processors on the same camera
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .addProcessor(colorSensor)
                .build();

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
    }
}
