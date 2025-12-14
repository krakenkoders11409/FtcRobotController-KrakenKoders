package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class Robot {

    public DriveSubsystem drive;
    //public IntakeSubsystem intake;
    public ShooterSubsystem shooter;

    public TurretSubsystem turret;

    public VisionSubsystem vision;

    public Robot(HardwareMap hardwareMap) {
        // Initialize all subsystems in a controlled, predictable order
        turret = new TurretSubsystem(hardwareMap);
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        //intake = new IntakeSubsystem(hardwareMap);
    }

    public void addTelemetry(Telemetry telemetry) {
        drive.addTelemetry(telemetry);
        shooter.addTelemetry(telemetry);
        vision.addTelemetry(telemetry);
        turret.addTelemetry(telemetry);
    }

    /** Called every control loop — optional, but great practice */
    public void periodic() {
        drive.periodic();
        shooter.update();
        vision.periodic();
        // Intake may not need periodic update
    }
}
