package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {
    // Setup for Turret Servo ------------------------------------------------------------
    private final int turretMax = 1200;
    private final int turretMin = 0;
    private int turretPos = 0;
    private final DcMotorEx turntableMotor;
    
    // Setup for Speed for both Turret and Angle -----------------------------------------
    private final double speed = 10;
    
    // ---------------------------------------------------------------
    public enum State {AUTO, IDLE, MANUAL}

    private final int horizontalTolerance = 10;
    private final int verticalTolerance = 10;

    private State state = State.IDLE;

    private double targetHorizontalPosition = 0;
    private double targetVerticalPosition = 0;

    private double currentHorizontalPosition = 0;
    private double currentVerticalPosition = 0;



    private boolean busy = false;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turntableMotor = hardwareMap.get(DcMotorEx.class, "turntableMotor");
        

        // Set directions (adjust if movement is inverted) ----------
        turntableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntableMotor.setTargetPosition(0);
        turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntableMotor.setPower(0.6);

      }
    // Manual Aiming
    public void manualAiming(double horizontal){
        turretPos += (int)(horizontal * speed);
        turretPos = Math.max(turretMin, Math.min(turretMax, turretPos));
        turntableMotor.setTargetPosition(turretPos);

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
        telemetry.addData("Turret position = ", turretPos);


    }

}

