package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class TurretSubsystem {
    private VisionSubsystem vision;
    // Setup for Turret Motor  ------------------------------------------------------------
    private final double turretMax = 0.85;
    private final double turretMin = 0.15;
    private double TurretPos = 0;
    // Setup for Speed for both Turret and Angle -----------------------------------------
    private final double speed = 1;
    double turnPower = 0;

    // Limelight aiming constants --------------------------------------------------------
    private static double TURRET_KP = 0.015; // tune on field (orig 0.015)

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

    public final DcMotor turntableMotor;

    // Limits (ticks)
    private static final int TURRET_MIN_TICKS = -320;
    private static final int TURRET_MAX_TICKS =  245;

    // Turret conversion: ticks <-> degrees
    // NOTE: replace with your turret's actual ticks-per-degree.
    private static final double TURRET_TICKS_PER_DEGREE = 7.75; // <-- tune/replace

    // Field-fallback PID
    private static final double FIELD_KP = 0.015;   // start small, tune on field
    private static final double FIELD_TOLERANCE_DEG = 1.0; // stop threshold deg

    // Feedforward (robot yaw rate compensation)
    private static final double kFF_DEFAULT = 0.02; // 0.01-0.03 recommended
    private double kFF = kFF_DEFAULT;
    private double robotYawRateDegPerSec = 0.0; // must be set by opmode each loop (see notes)

    // Tag field poses (meters). Replace with your actual AprilTag coordinates.
    // Keys: tag ID -> double[]{xMeters, yMeters}
    private final Map<Integer, double[]> TAG_POSES_METERS = new HashMap<Integer,double[]>() {{
        put(1, new double[] {1.0, 1.0});   // example: tag 1 at (1m, 1m)
        put(2, new double[] {3.0, 1.5});   // replace these values with your field map
        // add your tag positions...
    }};

    private State state = State.IDLE;

    // Horizontal aim offset (degrees)
    // + = shift right, - = shift left
    private double txOffsetDeg = 0.0;

    // Small timer to estimate dt if needed
    private final ElapsedTime timer = new ElapsedTime();
    private double lastLoopTimeSec = -1.0;

    private boolean busy = false;

    public TurretSubsystem(HardwareMap hardwareMap, Telemetry telemetry, VisionSubsystem vision) {
        this.vision = vision;
        turntableMotor = hardwareMap.get(DcMotor.class, "turntableMotor");
        if (turntableMotor == null) {
            throw new IllegalStateException("turntableMotor not found in hardware map!");
        }

        // Set Encoder Logic -------------------------------------------------
        turntableMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turntableMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turntableMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set directions (adjust if movement is inverted) ------------------
        turntableMotor.setDirection(DcMotor.Direction.FORWARD);

        // init timer
        timer.reset();
        lastLoopTimeSec = timer.seconds();
    }

    // State control -----------------------------------------------------
    public void enableAutoAim() {
        state = State.AUTO;
        autoAimEnabled = true;
    }

    public void disableAutoAim() {
        state = State.IDLE;
        autoAimEnabled = false;
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
        if (Math.abs(horizontal) < MANUAL_DEADZONE) {
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

    public void setTxOffset(double offset) {
        txOffsetDeg = offset;
    }

    public double getTxOffset() {
        return txOffsetDeg;
    }

    public void toggleAutoAim() {
        autoAimEnabled = !autoAimEnabled;
        if (!autoAimEnabled) {
            state = State.IDLE;
            turntableMotor.setPower(0);
        }
    }

    /** Set measured robot yaw rate in deg/sec (positive = turning right).
     *  Call each loop with IMU-derived rate. If not available, leave 0. */
    public void setRobotYawRateDegPerSec(double rate) {
        robotYawRateDegPerSec = rate;
    }

    /** Set feedforward gain (small; typical 0.01 - 0.03). */
    public void setFeedforwardGain(double k) {
        kFF = k;
    }

    /** Convert encoder ticks -> turret absolute angle in degrees.
     *  NOTE: This assumes 0 ticks corresponds to 0 degrees; adjust offset externally if needed. */
    private double turretAngleDegFromTicks(int ticks) {
        return ticks / TURRET_TICKS_PER_DEGREE;
    }

    /** Convert degrees -> encoder ticks */
    private int degreesToTicks(double deg) {
        return (int) Math.round(deg * TURRET_TICKS_PER_DEGREE);
    }

    private double normalizeDeg(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    /**
     * Field-based fallback aiming:
     * Compute desired turret angle (deg) given tag field coordinates and robot field pose.
     * Field units must match VisionSubsystem field units (meters).
     */
    private boolean tryFieldFallbackAim(Telemetry telemetry) {
        // Must have a field pose
        if (!vision.hasFieldPose()) return false;

        // Choose a tag to aim at:
        int activeTag = vision.getActiveTargetTag();
        int primaryTag = vision.getPrimaryTargetTag();

        int tagToUse = -1;
        if (activeTag >= 0 && TAG_POSES_METERS.containsKey(activeTag)) {
            tagToUse = activeTag;
        } else if (primaryTag >= 0 && TAG_POSES_METERS.containsKey(primaryTag)) {
            tagToUse = primaryTag;
        } else {
            // If no mapped tag found, do not attempt fallback
            return false;
        }

        double[] tagPose = TAG_POSES_METERS.get(tagToUse);
        double tagX = tagPose[0];
        double tagY = tagPose[1];

        double robotX = vision.getFieldX();
        double robotY = vision.getFieldY();
        double robotYaw = vision.getFieldYaw(); // degrees

        // desired angle in field space (deg) (0 = +X axis)
        double desiredFieldAngleRad = Math.atan2(tagY - robotY, tagX - robotX);
        double desiredFieldAngleDeg = Math.toDegrees(desiredFieldAngleRad);

        // desired turret angle relative to robot body
        double desiredTurretAngleDeg = normalizeDeg(desiredFieldAngleDeg - robotYaw);

        // current turret angle (from encoders)
        int currentTicks = turntableMotor.getCurrentPosition();
        double currentTurretDeg = turretAngleDegFromTicks(currentTicks);

        double errorDeg = normalizeDeg(desiredTurretAngleDeg - currentTurretDeg);

        // If within tolerance, stop
        if (Math.abs(errorDeg) <= FIELD_TOLERANCE_DEG) {
            turntableMotor.setPower(0);
            return true;
        }

        // P control
        double power = FIELD_KP * errorDeg;

        // clamp
        if (power > MAX_AUTO_POWER) power = MAX_AUTO_POWER;
        if (power < -MAX_AUTO_POWER) power = -MAX_AUTO_POWER;

        // enforce encoder hard limits
        if ((currentTicks <= TURRET_MIN_TICKS && power < 0) || (currentTicks >= TURRET_MAX_TICKS && power > 0)) {
            turntableMotor.setPower(0);
        } else {
            turntableMotor.setPower(power);
        }

        // telemetry helpful for debugging
        if (telemetry != null) {
            telemetry.addData("FIELD_TAG_ID", tagToUse);
            telemetry.addData("DES_TURRET_DEG", "%.2f", desiredTurretAngleDeg);
            telemetry.addData("CUR_TURRET_DEG", "%.2f", currentTurretDeg);
            telemetry.addData("TURRET_ERR_DEG", "%.2f", errorDeg);
            telemetry.addData("FIELD_POWER", "%.3f", power);
        }

        return true;
    }

    /**
     * Auto aim using Limelight TX (corrected). This re-uses your autoAim(...) logic but we
     * call it only once per loop and pass correctedTx (with offset + feedforward).
     */
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
        // clamp
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

    public void initAim(double xOffset, int targetTagID) {

        // Set turret TX offset to +10 degrees <- Left (adjust if your sign convention differs)
        setTxOffset(xOffset);
        setTurretKP(0.03);

        // Prefer explicit enable if available. If not, toggle is a fallback.
        try {
            // if API has enableAutoAim(boolean)
            getClass().getMethod("enableAutoAim", boolean.class).invoke(true);
        } catch (Exception e) {
            // fallback to toggle if explicit method not present
            toggleAutoAim();
        }

        vision.clearAllowedTags();
        vision.addAllowedTag(targetTagID);

    }

    /**
     * Main update loop. Call this every robot loop.
     *
     * horizontalInput -> driver joystick (-1..1). Manual > Auto > Idle priority enforced.
     * tx, hasTarget are optional inputs you already computed from vision, but we prefer
     * to read authoritative values from VisionSubsystem when available.
     *
     * NOTE: Provide robot yaw rate by calling setRobotYawRateDegPerSec(...) from your OpMode.
     */
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
        state = State.AUTO;

        // Prefer using VisionSubsystem values if present (keeps call-sites simpler)
        boolean visionHasTarget = vision.hasTarget();
        double rawTx = vision.getTx();

        // apply configured tx offset
        double correctedTx = rawTx + txOffsetDeg;

        // feedforward based on yaw rate — MUST be set by robot loop via setRobotYawRateDegPerSec(...)
        double ff = -kFF * robotYawRateDegPerSec;
        correctedTx += ff;

        // If vision target available -> use limelight auto aim
        if (visionHasTarget) {
            autoAim(correctedTx, true);
            return;
        }

        // If vision lost, attempt field-pose fallback (must have field pose and a known tag position)
        if (vision.hasFieldPose()) {
            boolean used = tryFieldFallbackAim(null);
            if (used) return;
        }

        // Otherwise, hold position (no vision, no field) — stop motor
        turntableMotor.setPower(0);
    }

    public void addTelemetry (Telemetry telemetry){
        telemetry.addLine("----- Turret -----");
        telemetry.addData("Turret State = ", state);

        // Mode hint
        String mode = "IDLE";
        if (state == State.MANUAL) mode = "MANUAL";
        else if (state == State.AUTO) {
            if (vision.hasTarget()) mode = "VISION";
            else if (vision.hasFieldPose()) mode = "FIELD";
            else mode = "AUTO";
        }
        telemetry.addData("Mode", mode);

        telemetry.addData("Turret Horizontal Power = ", turntableMotor.getPower());
        telemetry.addData("Turret Encoder", turntableMotor.getCurrentPosition());
        telemetry.addData("At Min", turntableMotor.getCurrentPosition() <= TURRET_MIN_TICKS);
        telemetry.addData("At Max", turntableMotor.getCurrentPosition() >= TURRET_MAX_TICKS);

        telemetry.addData("Vision HasTarget", vision.hasTarget());
        telemetry.addData("Vision HasFieldPose", vision.hasFieldPose());
        telemetry.addData("Vision TX", vision.getTx());
        telemetry.addData("Active Tag", vision.getActiveTargetTag());
        telemetry.addData("Primary Tag", vision.getPrimaryTargetTag());

        telemetry.addData("FF k", kFF);
        telemetry.addData("YawRate (deg/s)", robotYawRateDegPerSec);
    }

}
