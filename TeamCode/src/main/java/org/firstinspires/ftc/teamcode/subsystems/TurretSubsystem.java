package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {
    public enum State {AUTO, IDLE, MANUAL}

    public final DcMotorEx null_motor;
    public final Servo null_servo, null_servo_2;
    private final int horizontalTolerance = 10;
    private final int verticalTolerance = 10;

    private State state = State.IDLE;

    private double targetHorizontalPosition = 0;
    private double targetVerticalPosition = 0;

    private double currentHorizontalPosition = 0;
    private double currentVerticalPosition = 0;

    private boolean busy = false;

    public TurretSubsystem(HardwareMap hardwareMap) {
        null_motor = hardwareMap.get(DcMotorEx.class, "null");
        null_servo = hardwareMap.get(Servo.class, "null_servo");
        null_servo_2 = hardwareMap.get(Servo.class, "null_servo_2");

        // Encoder logic -------------------------------------------------
        null_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        null_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set directions (adjust if movement is inverted) ----------
        null_motor.setDirection(DcMotorEx.Direction.REVERSE);
        null_servo.setDirection(Servo.Direction.REVERSE);
        null_servo_2.setDirection(Servo.Direction.REVERSE);


        // Set motor behavior ----------------------------------------------
        null_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }
    // Tele-Op ----------------------------------------------

    // Manual Aiming with driver input
    public void manualAiming(double horizontalPower, double verticalDelta) {

        state = State.MANUAL;
        // Horizontal turret is a motor
        null_motor.setPower(horizontalPower);

        // Vertical turret is a servo              Small change
        double newPos = null_servo.getPosition() + verticalDelta;

        // Limit so servos don't break
        newPos = Math.max(0.0, Math.min(1.0, newPos));

        null_servo.setPosition(newPos);
    }
    // Auto ----------------------------------------------

    // Look For Game Objects
    public void lookForGameObjects() {
        if (busy) {
            return;
        }
        state = State.AUTO;
        busy = true;

        null_motor.setTargetPositionTolerance(horizontalTolerance);
        null_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        currentHorizontalPosition = null_motor.getCurrentPosition();
        currentVerticalPosition = null_servo.getPosition();


        targetHorizontalPosition = 0;
        targetVerticalPosition = 0;
        if (targetHorizontalPosition > 0){
            while (targetHorizontalPosition != currentHorizontalPosition) {
                null_motor.setPower(0.5);
            }
        } else {
            while (targetHorizontalPosition != currentHorizontalPosition) {
                null_motor.setPower(-0.5);
            }
        }
        null_servo.setPosition(targetVerticalPosition);
        busy = false;
    }

    // PID ----------------------------------------------

    // PID Variables
    private double kP = 0.02;
    private double kI = 0.0;
    private double kD = 0.001;
    private double integral = 0;
    private double lastError = 0;

    private double pid(double error) {
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        return kP * error + kI * integral + kD * derivative;

    }
    // PID April Tag Tracking
    public void aprilTagLock(double tx, boolean hasTarget) {
        if (!hasTarget) {
            null_motor.setPower(0);
            return;
        }
        state = State.AUTO;
        busy = true;


        // Horizontal Control (Motor - TurnTable)
        double horizontalError = tx;
        double motorPower = pid(horizontalError);

        // Power Limit
        motorPower = Math.max(-0.4, Math.min(0.4, motorPower));
        null_motor.setPower(motorPower);

        busy = false;
    }








    public void addTelemetry (Telemetry telemetry){
        telemetry.addLine("----- Turret -----");
        telemetry.addData("Turret State = ", state);
        telemetry.addData("Turret Horizontal Position = ", currentHorizontalPosition);
        telemetry.addData("Turret Vertical Position = ", currentVerticalPosition);


    }

}

