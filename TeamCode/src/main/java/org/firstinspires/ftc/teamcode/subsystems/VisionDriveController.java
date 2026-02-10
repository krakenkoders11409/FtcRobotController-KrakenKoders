package org.firstinspires.ftc.teamcode.auton;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class VisionDriveController {

    private final DriveSubsystem drive;
    private final VisionSubsystem vision;

    // Simple P gains (start here)
    private static final double KP_X = 0.6;     // forward
    private static final double KP_Y = 0.6;     // strafe
    private static final double KP_H = 0.02;    // turn

    // Limits
    private static final double MAX_DRIVE = 0.6;
    private static final double MAX_TURN  = 0.5;

    // Target pose (meters, degrees)
    private double targetX, targetY, targetHeading;

    public VisionDriveController(DriveSubsystem drive, VisionSubsystem vision) {
        this.drive = drive;
        this.vision = vision;
    }

    public void setTarget(double xMeters, double yMeters, double headingDeg) {
        targetX = xMeters;
        targetY = yMeters;
        targetHeading = headingDeg;
    }

    public boolean update() {
        if (!vision.hasFieldPose()) {
            drive.stop();
            return false;
        }

        double xError = targetX - vision.getFieldX();
        double yError = targetY - vision.getFieldY();
        double hError = angleWrap(targetHeading - vision.getFieldYaw());

        double forward = clamp(xError * KP_X, MAX_DRIVE);
        double strafe  = clamp(yError * KP_Y, MAX_DRIVE);
        double turn    = clamp(hError * KP_H, MAX_TURN);

        double confidence = vision.getPoseConfidenceScale();

        if (confidence <= 0.0) {
            drive.stop();
            return false;
        }

        forward *= confidence;
        strafe  *= confidence;
        turn    *= confidence;

        drive.drive(forward, strafe, turn);


        return isAtTarget(xError, yError, hError);
    }

    private boolean isAtTarget(double xErr, double yErr, double hErr) {
        return Math.abs(xErr) < 0.05   // 5 cm
                && Math.abs(yErr) < 0.05
                && Math.abs(hErr) < 3.0;   // degrees
    }

    private double clamp(double v, double max) {
        return Math.max(-max, Math.min(max, v));
    }

    private double angleWrap(double deg) {
        while (deg > 180) deg -= 360;
        while (deg < -180) deg += 360;
        return deg;
    }
}
