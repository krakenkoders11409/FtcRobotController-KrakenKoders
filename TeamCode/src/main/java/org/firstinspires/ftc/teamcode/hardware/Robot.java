package org.firstinspires.ftc.teamcode.hardware;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class Robot {

    public DriveSubsystem drive;
    public ShooterSubsystem shooter;
    public TurretSubsystem turret;
    public VisionSubsystem vision;

    private final IMU imu;
    private Follower follower;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize all subsystems in a controlled, predictable order

        // --- Subsystems ---
        turret = new TurretSubsystem(hardwareMap, telemetry);
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        imu = hardwareMap.get(IMU.class, "imu");
        //intake = new IntakeSubsystem(hardwareMap);

        // --- Vision Hardware ---
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void addTelemetry(Telemetry telemetry) {
        drive.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        turret.addTelemetry(telemetry);
        vision.addTelemetry(telemetry);
    }

    /** Called every control loop — optional, but great practice */
    public void update() {
        // --- Update Subsystems
        drive.update();
        shooter.update();
        vision.update();

        // Read Vision output
        double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        vision.updateRobotOrientation(headingDeg);

    }

    /**
     * Initialize the Pedro Pathing Follower for teleop field-oriented driving.
     * Call this before waitForStart() in teleop.
     */
    public void initFollowerForTeleop(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleopDrive();
    }

    public Follower getFollower() {
        return follower;
    }

    /**
     * Get the robot's current heading in radians.
     * Uses Follower (sensor-fused) if available, falls back to IMU.
     */
    public double getHeadingRadians() {
        if (follower != null) {
            return follower.getPose().getHeading();
        }
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /**
     * Reset the heading so the current direction becomes "forward" (0 radians).
     */
    public void resetHeading() {
        if (follower != null) {
            Pose current = follower.getPose();
            follower.setStartingPose(new Pose(current.getX(), current.getY(), 0));
        }
        imu.resetYaw();
    }
}
