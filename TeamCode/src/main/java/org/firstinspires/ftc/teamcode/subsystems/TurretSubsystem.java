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
    private final double speed = 1;
    double turnPower = 0;

    // Limelight aiming constants --------------------------------------------------------
    private static double TURRET_KP = 0.06; // tune on field orig 0.015

    public void setTurretKP(double kp) {
        TURRET_KP = kp;
    }

    private static final double MAX_AUTO_POWER = 1;
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
    public void turretRunToPosition(int targetTicks) {
        if (state == State.MANUAL) {
            return; // do not override manual control
        }

        // Set the target
        turntableMotor.setTargetPosition(targetTicks);

        // Switch mode
        turntableMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Apply power (FTC SDK requires power > 0 to move)
        turntableMotor.setPower(0.5); // adjust speed as needed
    }

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

    // Horizontal aim offset (degrees)
    // + = shift right, - = shift left
    private double txOffsetDeg = 0.0;

    public void setTxOffset(double offset) {
        txOffsetDeg = offset;
    }

    public double getTxOffset() {
        return txOffsetDeg;
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

        double power = tx * (TURRET_KP * -1);
        power = Math.max(-MAX_AUTO_POWER, Math.min(MAX_AUTO_POWER, power));

        int currentPos = turntableMotor.getCurrentPosition();

        if ((currentPos <= TURRET_MIN_TICKS && power < 0) || (currentPos >= TURRET_MAX_TICKS && power > 0)) {
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

        // 1. Manual ALWAYS wins
        if (Math.abs(horizontalInput) > MANUAL_DEADZONE) {
            manualOverride = true;
            autoAimEnabled = false;       // force-disable auto aim
            state = State.MANUAL;
            manualAiming(horizontalInput);
            return;
        }

        // 2. Stick returned to center → clear override
        if (manualOverride && Math.abs(horizontalInput) <= MANUAL_DEADZONE) {
            manualOverride = false;
        }

        // 3. If auto aim is OFF → go idle
        if (!autoAimEnabled) {
            state = State.IDLE;
            turntableMotor.setPower(0);
            return;
        }

        // 4. Auto aim behavior
        double correctedTx = tx + txOffsetDeg;  // apply your limelight offset here
        state = State.AUTO;
        autoAim(correctedTx, hasTarget);
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

