package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {

    public enum State { IDLE, MANUAL, MOVING_TO_TARGET, HOMING }
    public String debugging = "";

    private final DcMotor turntableMotor;

    // “Soft limits” in encoder ticks (tune these!)
    private final int turretMinTicks = -1200;
    private final int turretMaxTicks =  1200;

    // The encoder tick that represents “straight ahead”
    // If you add a home sensor later, you’ll set this after homing.
    private int forwardTicks = 0;

    private State state = State.IDLE;

    // Cached values for telemetry/debug
    private int targetTicks = 0;
    private int currentTicks = 0;

   public TurretSubsystem(HardwareMap hardwareMap) {
        turntableMotor = hardwareMap.get(DcMotor.class, "turntableMotor");
        

        // Set directions (adjust if movement is inverted) ----------
        turntableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntableMotor.setTargetPosition(0);
        turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turntableMotor.setPower(0.6);

      }
    // Manual Aiming
    public void manualAiming(double joystickX){
        state = State.MANUAL;

        // deadband
        if (Math.abs(joystickX) < 0.05) {
            turntableMotor.setPower(0);
            if (state == State.MANUAL) state = State.IDLE;
            return;
        }

        // Safety: don’t drive past soft limits
        int pos = turntableMotor.getCurrentPosition();
        if ((pos <= turretMinTicks && joystickX < 0) || (pos >= turretMaxTicks && joystickX > 0)) {
            debugging = "inside soft limits";
            turntableMotor.setPower(0);
            return;
        }

        // Power scale (tune!)
        double power = joystickX * 0.6; // try 0.2–0.6 depending on gearing
        turntableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turntableMotor.setPower(power);
    }

    /**
     * Move to an absolute encoder tick target (Auto-friendly).
     */
    public void goToTicks(int ticks, double maxPower) {
        targetTicks = clamp(ticks, turretMinTicks, turretMaxTicks);

        turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turntableMotor.setTargetPosition(targetTicks);

        // IMPORTANT: apply power after setting the target
        turntableMotor.setPower(Math.abs(maxPower));

        state = State.MOVING_TO_TARGET;
    }

    /**
     * If you want to “define straight ahead” at runtime (ex: in init),
     * you can do this AFTER you manually align the turret.
     */
    public void setCurrentAsForward() {
        forwardTicks = turntableMotor.getCurrentPosition();
    }

    public int getCurrentTicks() {
        return turntableMotor.getCurrentPosition();
    }

    /**
     * “Point straight ahead” (Auto).
     */
    public void pointForward() {
        goToTicks(forwardTicks, 0.5);
    }

    /**
     * Call every loop (TeleOp + Auto)
     */
    public void periodic() {
        currentTicks = turntableMotor.getCurrentPosition();

        // If we’re moving to a target, detect when we’re basically “done”
        if (state == State.MOVING_TO_TARGET) {
            if (!turntableMotor.isBusy() || Math.abs(targetTicks - currentTicks) < 10) {
                // Stop holding power once done (or keep a tiny hold power if you want)
                turntableMotor.setPower(0);
                state = State.IDLE;
            }
        }
    }


    public void addTelemetry (Telemetry telemetry){
        telemetry.addLine("----- Turret -----");
        telemetry.addData("State", state);
        telemetry.addData("CurrentTicks", turntableMotor.getCurrentPosition());
        telemetry.addData("debugging", debugging);


    }

    private static int clamp(int v, int min, int max) {
        return Math.max(min, Math.min(max, v));
    }

}

