package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {
    // Setup for Turret Servo ------------------------------------------------------------
    private final double turretMax = 0.85;
    private final double turretMin = 0.15;
    private double TurretPos = 0;
    // Setup for Speed for both Turret and Angle -----------------------------------------
    private final double speed = 0.8;
    
    



    public enum State {AUTO, IDLE, MANUAL}

    private final CRServo turntableServo;
    private final int horizontalTolerance = 10;
    private final int verticalTolerance = 10;

    private State state = State.IDLE;

    private double targetHorizontalPosition = 0;
    private double targetVerticalPosition = 0;

    private double currentHorizontalPosition = 0;
    private double currentVerticalPosition = 0;



    private boolean busy = false;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        turntableServo = hardwareMap.get(CRServo.class, "turntableServo");


        // Set directions (adjust if movement is inverted) ----------
        turntableServo.setDirection(CRServo.Direction.FORWARD);


      }
    // Manual Aiming
    public void manualAiming(double horizontal){
        double turnPower = horizontal * speed;
        turntableServo.setPower(turnPower);
    }
    
    // Look For Game Objects
    public void lookForGameObjects() {
        if (busy) {
            return;
        }
        state = State.AUTO;
        busy = true;


        currentHorizontalPosition = turntableServo.getPower();

//        targetHorizontalPosition = 0;
//        targetVerticalPosition = 0;
//        if (targetHorizontalPosition > 0){
//            while (targetHorizontalPosition != currentHorizontalPosition) {
//                //null_motor.setPower(0.5);
//            }
//        } else {
//            while (targetHorizontalPosition != currentHorizontalPosition) {
//                //null_motor.setPower(-0.5);
//            }
//        }
//        lefVerticalServo.setPosition(targetVerticalPosition);
    }


    public void addTelemetry (Telemetry telemetry){
        telemetry.addLine("----- Turret -----");
        telemetry.addData("Turret State = ", state);
        telemetry.addData("Turret Horizontal Position = ", currentHorizontalPosition);
        telemetry.addData("Turret Vertical Position = ", currentVerticalPosition);


    }

}

