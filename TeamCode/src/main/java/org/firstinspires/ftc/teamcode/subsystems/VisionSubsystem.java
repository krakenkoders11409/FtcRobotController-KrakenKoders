package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VisionSubsystem {
    private final Limelight3A limelight;

    // Raw Limelight values
    private boolean hasTarget = false;

    private double robotYawDeg = 0.0;
    private double tx;    // horizontal offset
    private double ty;    // vertical offset
    private double ta;    // target area

    // Lime Light Instance Builder
    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Poll up to 100x per second for fresh data
        limelight.setPollRateHz(100);

        // Start with pipeline 0 (configure in Limelight UI)
        limelight.pipelineSwitch(0);

        // Start background polling thread
        limelight.start();
    }

    // --- Change pipelines if you add more in the web UI. ---
    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }
    /**
     * Simple steering correction value you can feed into your drive turn value.
     * Tunable proportional gain (kP).
     */

    public void updateRobotOrientation(double robotHeadingDeg) {
        // FTC IMU is CW-positive
        // Limelight expects CCW-positive;
        robotYawDeg = robotHeadingDeg;

        double limelightYawDeg = -robotHeadingDeg;
        limelight.updateRobotOrientation(limelightYawDeg);
    }


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
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- Vision -----");
        telemetry.addData("Robot Yaw (deg)", robotYawDeg);
        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("ta", ta);
    }


}
