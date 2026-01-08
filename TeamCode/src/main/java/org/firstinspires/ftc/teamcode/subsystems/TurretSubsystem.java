package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;



import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem {
    // Setup for Turret Motor  ------------------------------------------------------------
    private final double turretMax = 0.85;
    private final double turretMin = 0.15;
    private double TurretPos = 0;
    // Setup for Speed for both Turret and Angle -----------------------------------------
    private final double speed = 0.8;
    double turnPower = 0;

    // Limelight aiming constants --------------------------------------------------------
    private static final double TURRET_KP = 0.5; // tune on field
    private static final double MAX_AUTO_POWER = 0.5;
    private static final double TX_DEADBAND = 1.0; // degrees

    // Aiming ------------------------------------------------------------------------------
    public enum State {
        IDLE,
        MANUAL,
        AUTO
    }
    private boolean autoAimEnabled = false;  // toggled by X
    private boolean manualOverride = false;   // true while joystick moved
    private final double MANUAL_DEADZONE = 0.05;





    private final DcMotor turntableMotor;
    private final int horizontalTolerance = 10;
    private final int verticalTolerance = 10;

    private State state = State.IDLE;

    private double targetHorizontalPosition = 0;
    private double targetVerticalPosition = 0;

    private double currentHorizontalPosition = 0;
    private double currentVerticalPosition = 0;

    // Limits
    private static final int TURRET_MIN_TICKS = -320;
    private static final int TURRET_MAX_TICKS =  245;




    private boolean busy = false;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        turntableMotor = hardwareMap.get(DcMotor.class, "turntableMotor");

        // Set Encoder Logic -------------------------------------------------
        turntableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set directions (adjust if movement is inverted) ----------
        turntableMotor.setDirection(DcMotor.Direction.FORWARD);
      }

      // State control -----------------------------------------------------
      public void enableAutoAim() {
          state = State.AUTO;
      }

    public void disableAutoAim() {
        state = State.IDLE;
        turntableMotor.setPower(0);
    }

    public State getState() {
        return state;
    }



    // Manual Aiming
    public void manualAiming(double horizontal) {

        // Deadzone
        if (Math.abs(horizontal) < 0.05) {
            if (state == State.MANUAL) {
                turntableMotor.setPower(0);
            }
            return;
        }

        // Manual input ALWAYS overrides auto
        state = State.MANUAL;

        int currentPos = turntableMotor.getCurrentPosition();
        double power = horizontal * speed;

        if ((currentPos <= TURRET_MIN_TICKS && power < 0) ||
                (currentPos >= TURRET_MAX_TICKS && power > 0)) {
            turntableMotor.setPower(0);
        } else {
            turntableMotor.setPower(power);
        }
    }


    public void toggleAutoAim() {
        autoAimEnabled = !autoAimEnabled;
    }
    public void autoAim(double tx, boolean hasTarget) {

        if (state != State.AUTO) {
            return; // auto aim not allowed
        }

        if (!hasTarget) {
            turntableMotor.setPower(0);
            return;
        }

        if (Math.abs(tx) < TX_DEADBAND) {
            turntableMotor.setPower(0);
            return;
        }

        double power = tx * TURRET_KP;
        power = Math.max(-MAX_AUTO_POWER, Math.min(MAX_AUTO_POWER, power));

        int currentPos = turntableMotor.getCurrentPosition();

        if ((currentPos <= TURRET_MIN_TICKS && power < 0) ||
                (currentPos >= TURRET_MAX_TICKS && power > 0)) {
            turntableMotor.setPower(0);
        } else {
            turntableMotor.setPower(power);
        }
    }




    // Look For Game Objects
    public void lookForGameObjects() {
        if (busy) {
            return;
        }
        state = State.AUTO;
        busy = true;

    }
    public void update(double horizontalInput, double tx, boolean hasTarget) {
        // Detect manual override
        if (Math.abs(horizontalInput) > MANUAL_DEADZONE) {
            manualOverride = true;
        } else if (manualOverride) {
            manualOverride = false;  // release override when stick returns to center
        }

        // Decide what to run
        if (autoAimEnabled && !manualOverride && hasTarget) {
            autoAim(tx, true);
        } else {
            manualAiming(horizontalInput);
        }
    }


    public void addTelemetry (Telemetry telemetry){
        telemetry.addLine("----- Turret -----");
        telemetry.addData("Turret State = ", state);
        telemetry.addData("Turret Horizontal Power = ", turntableMotor.getPower());
        telemetry.addData("Turret Encoder", turntableMotor.getCurrentPosition());
        telemetry.addData("At Min", turntableMotor.getCurrentPosition() <= TURRET_MIN_TICKS);
        telemetry.addData("At Max", turntableMotor.getCurrentPosition() >= TURRET_MAX_TICKS);



    }

}

