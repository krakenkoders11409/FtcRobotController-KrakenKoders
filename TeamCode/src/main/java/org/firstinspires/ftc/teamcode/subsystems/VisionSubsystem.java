package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class VisionSubsystem {
    private final Limelight3A limelight;

    // Raw Limelight values
    public boolean hasTarget = false;
    public boolean hasFieldPose = false;


    // Tag offsets
    private double tx;    // horizontal offset
    private double ty;    // vertical offset
    private double ta;    // target area

    // Field pose (meters, degrees)
    private double fieldX = 0.0;
    private double fieldY = 0.0;
    private double fieldYaw = 0.0;
    // Robot Yaw
    private double robotYawDeg = 0.0;

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

    // --- Field Data ---
    public void updateRobotOrientation(double robotHeadingDeg) {
        robotYawDeg = robotHeadingDeg;

        double limelightYawDeg = -robotHeadingDeg;
        limelight.updateRobotOrientation(limelightYawDeg);
    }

    public double getFieldX() {
        return fieldX; }
    public double getFieldY() {
        return fieldY; }
    public double getFieldYaw() {
        return fieldYaw; }
    public boolean hasFieldPose() {
        return hasFieldPose;
    }

    // --- Target Data Functions ---
    public boolean hasTarget() {
        return hasTarget;
    }
    public double getTx() {
        return tx;
    }




    public void periodic() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            hasTarget = true;

            // 2D offsets
            tx = result.getTx();
            ty = result.getTy();
            ta = result.getTa();

            // ---- 3D robot pose from AprilTags ----
            // Try the MT2 (MegaTag2) fused pose first:
            Pose3D botposeMT2 = result.getBotpose_MT2();
            if (botposeMT2 != null) {
                hasFieldPose = true;
                fieldX = botposeMT2.getPosition().x;
                fieldY = botposeMT2.getPosition().y;
                // yaw is the rotation about vertical axis
                fieldYaw = botposeMT2.getOrientation().getYaw(AngleUnit.DEGREES);
            } else {
                // Fall back to basic botpose if MT2 is unavailable
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    hasFieldPose = true;
                    fieldX = botpose.getPosition().x;
                    fieldY = botpose.getPosition().y;
                    fieldYaw = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
                } else {
                    hasFieldPose = false;
                }
            }
        } else {
            hasTarget = false;
            hasFieldPose = false;
        }
    }




    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- Vision -----");
        telemetry.addData("Robot Yaw (deg)", robotYawDeg);
        telemetry.addData("Has Target", hasTarget);
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("ta", ta);
        telemetry.addLine("");
        telemetry.addData("Field X (m)", fieldX);
        telemetry.addData("Field Y (m)", fieldY);
        telemetry.addData("Field Yaw (deg)","%.1f°", fieldYaw);
    }
}
