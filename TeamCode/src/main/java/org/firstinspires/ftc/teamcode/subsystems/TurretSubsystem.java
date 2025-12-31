package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;



import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {
    // Setup for Turret Servo ------------------------------------------------------------
    private final double turretMax = 0.85;
    private final double turretMin = 0.15;
    private double TurretPos = 0;
    // Setup for Speed for both Turret and Angle -----------------------------------------
    private final double speed = 0.8;
    double turnPower = 0;
    
    



    public enum State {AUTO, IDLE, MANUAL}

    private final DcMotor turntableMotor;
    private final int horizontalTolerance = 10;
    private final int verticalTolerance = 10;

    private State state = State.IDLE;

    private double targetHorizontalPosition = 0;
    private double targetVerticalPosition = 0;

    private double currentHorizontalPosition = 0;
    private double currentVerticalPosition = 0;



    private boolean busy = false;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        turntableMotor = hardwareMap.get(DcMotor.class, "turntableMotor");

        // Set directions (adjust if movement is inverted) ----------
        turntableMotor.setDirection(DcMotor.Direction.REVERSE);
      }
    // Manual Aiming
    public void manualAiming(double horizontal){
        turnPower = horizontal * speed;
        turntableMotor.setPower(turnPower);
    }
    
    // Look For Game Objects
    public void lookForGameObjects() {
        if (busy) {
            return;
        }
        state = State.AUTO;
        busy = true;

    }


    public void addTelemetry (Telemetry telemetry){
        telemetry.addLine("----- Turret -----");
        telemetry.addData("Turret State = ", state);
        telemetry.addData("Turret Horizontal Power = ", turntableMotor.getPower());


    }

}

