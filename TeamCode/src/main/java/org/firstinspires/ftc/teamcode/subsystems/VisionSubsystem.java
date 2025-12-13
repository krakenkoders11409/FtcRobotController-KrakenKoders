package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

public class LimeLightSubsystem {
    private final Limelight3A limelight;

    // Raw Limelight values
    private boolean hasTarget = false;
    private double tx;    // horizontal offset
    private double ty;    // vertical offset
    private double ta;    // target area
    private boolean tv;   // valid target?

    // Control flags
    private boolean aimAssistEnabled = false;

    // Lime Light Instance Builder
    public LimeLightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Poll up to 100x per second for fresh data
        limelight.setPollRateHz(100);

        // Start with pipeline 0 (configure in Limelight UI)
        limelight.pipelineSwitch(0);

        // Start background polling thread
        limelight.start();


    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- Vision -----");
        telemetry.addData("Target X", tx);
        telemetry.addData("Target Y", ty);
        telemetry.addData("Target Area", ta);
        telemetry.addData("hasTarget = ", hasTarget);
    }

    public void periodic() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            hasTarget = true;
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();
        } else {
            hasTarget = false;
        }
    }

    // --- Public API for rest of robot ---

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getTa() {
        return ta;
    }

    /**
     * Simple steering correction value you can feed into your drive turn value.
     * Tunable proportional gain (kP).
     */
    public double getSteeringCorrection() {
        if (!hasTarget) return 0.0;

        double kP = 0.03;  // start small and tune
        double correction = kP * tx;

        // Optional clamp so it doesn’t spin wildly
        double max = 0.3;
        if (correction > max) correction = max;
        if (correction < -max) correction = -max;

        return correction;
    }

    /** Change pipelines if you add more in the web UI. */
    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }
}
