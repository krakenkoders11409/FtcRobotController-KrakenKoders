package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
@Disabled
@TeleOp(name = "UtilityCamera (Blocks to Java)")
public class UtilityCamera extends LinearOpMode {

  /**
   * This OpMode helps calibrate a webcam or RC phone camera, useful for AprilTag pose estimation
   * with the FTC VisionPortal. It captures a camera frame (image) and stores it on the Robot Controller
   * (Control Hub or RC phone), with each press of the gamepad button X (or Square).
   * Full calibration instructions are here: https://ftc-docs.firstinspires.org/camera-calibration
   */
  @Override
  
  public void runOpMode() {
    VisionPortal.Builder myVisionPortalBuilder;
    boolean USE_WEBCAM;
    int RESOLUTION_WIDTH;
    int RESOLUTION_HEIGHT;
    VisionPortal myVisionPortal;
    boolean lastX;
    int frameCount;
    long capReqTime;
    boolean x;

    // EDIT THESE PARAMETERS AS NEEDED
    USE_WEBCAM = true;
    RESOLUTION_WIDTH = 1280;
    RESOLUTION_HEIGHT = 720;
    // Create a VisionPortal.Builder and set attributes related to the camera.
    myVisionPortalBuilder = new VisionPortal.Builder();
    if (USE_WEBCAM) {
      // Use a webcam.
      myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      // Use the device's back camera.
      myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    myVisionPortalBuilder.setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT));
    // Create a VisionPortal by calling build.
    myVisionPortal = myVisionPortalBuilder.build();
    lastX = false;
    frameCount = 0;
    capReqTime = 0;
    while (!isStopRequested()) {
      x = gamepad1.x;
      if (x && !lastX) {
        // Save the next frame to the given file.
        myVisionPortal.saveNextFrameRaw("CameraFrameCapture-" + frameCount);
        frameCount += 1;
        // Get the current time in milliseconds. The value returned represents
        // the number of milliseconds since midnight, January 1, 1970 UTC.
        capReqTime = System.currentTimeMillis();
      }
      lastX = x;
      telemetry.addLine("######## Camera Capture Utility ########");
      telemetry.addLine(" > Resolution: " + RESOLUTION_WIDTH + "x" + RESOLUTION_HEIGHT);
      telemetry.addLine(" > Press X (or Square) to capture a frame");
      telemetry.addData("> Camera Status", myVisionPortal.getCameraState());
      if (capReqTime != 0) {
        telemetry.addLine("");
        telemetry.addLine("Captured Frame!");
      }
      if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 1000) {
        capReqTime = 0;
      }
      telemetry.update();
    }
  }
}


/*
Focals (pixels) - Fx: 2089.86 Fy: 2089.86
Optical center - Cx: 784.734 Cy: 334.19
Radial distortion (Brown's Model)
K1: 0.123775 K2: 1.38478 K3: -10.2716
P1: -0.0174978 P2: 0.0356678
Skew: 0

Mean Square Reprojection Error: 0.444143 pixels

*/
