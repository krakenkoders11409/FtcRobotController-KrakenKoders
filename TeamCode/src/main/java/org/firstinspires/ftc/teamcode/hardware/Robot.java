package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
    public void UpdatePeriodic() {
        // --- Update Subsystems
        drive.periodic();
        shooter.update();
        vision.periodic();

        // Read Vision output
        double headingDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        vision.updateRobotOrientation(headingDeg);
    }
}
