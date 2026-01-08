package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class ShooterSubsystem {
    public enum State { IDLE, PRIME_INTAKE, PRIME_OUTTAKE, SPIN_UP, FIRE_BALLS, SPIN_DOWN_ALL }

    // Angling servos
    private static final double ANGLE_MIN = 0.35;
    private static final double ANGLE_MAX = 0.9;
    private static final double ANGLE_STEP = 0.02;
    private double anglePos = 0.5;

    private final DcMotor intake;
    private final Servo intakeBlockServo;
    private final DcMotorEx outtakeMotor;
    private final Servo leftVerticalServo;

    // Tunables
    private final int shortShotVelocity = 1400;
    private final int longShotVelocity = 2000;
    private final int ejectVelocity = 1000;
    private final double intakePower = 1.0;
    private final long primeIntakeMs = 900;
    private final long spinUpMs = 2000;
    private final long feedMs = 1000;
    private final long spinDownMs = 1000;
    private final int velocityTolerance = 35;

    private final ElapsedTime timer = new ElapsedTime();
    private State state = State.IDLE;
    private boolean busy = false;

    private int numberOfShots = 0;     // remaining shots to fire
    private String shotType = "";
    private double lastVelocity = 0;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeBlockServo = hardwareMap.get(Servo.class, "intakeBlockServo");
        leftVerticalServo = hardwareMap.get(Servo.class, "leftVerticalServo");

        // Encoder logic using DcMotor enums
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Directions
        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intakeBlockServo.setDirection(Servo.Direction.REVERSE);
        leftVerticalServo.setDirection(Servo.Direction.REVERSE);

        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ---------- State entry helper ----------
    private void enterState(State newState) {
        state = newState;
        timer.reset();
    }

    /** Start a single, timed shot. Returns immediately (non-blocking). */
    public void startShot(int shots, String type) {
        if (busy) return;

        numberOfShots = Math.max(0, shots);
        shotType = type == null ? "" : type;
        busy = true;
        // begin by priming intake
        enterState(State.PRIME_INTAKE);
        intake.setPower(-0.5); // prime direction as before
    }

    /** Call this every loop to advance the sequence without blocking. */
    public void update() {
        lastVelocity = outtakeMotor.getVelocity();

        switch (state) {

            case IDLE:
                intake.setPower(0);
                outtakeMotor.setPower(0);
                break;


            case PRIME_INTAKE:
                // intake reverse to seat ball
                intake.setPower(-0.5);
                if (timer.milliseconds() >= primeIntakeMs) {
                    intake.setPower(0);
                    enterState(State.PRIME_OUTTAKE);
                    // start bumping shooter backward
                    outtakeMotor.setPower(-0.5);
                }
                break;


            case PRIME_OUTTAKE:
                // brief backward bump of shooter flywheel
                if (timer.milliseconds() >= 400) {  // 300ms is typical; adjust as needed
                    outtakeMotor.setPower(0);
                    enterState(State.SPIN_UP);

                    // now start spinning up
                    if (Objects.equals(shotType, "short")) {
                        outtakeMotor.setVelocity(shortShotVelocity);
                    } else {
                        outtakeMotor.setVelocity(longShotVelocity);
                    }
                }
                break;


            case SPIN_UP:
                int targetVel = Objects.equals(shotType, "short")
                        ? shortShotVelocity
                        : longShotVelocity;

                double error = Math.abs(lastVelocity - targetVel);

                if (error <= velocityTolerance || timer.milliseconds() >= spinUpMs) {
                    enterState(State.FIRE_BALLS);
                    intake.setPower(1.0);
                }
                break;


            case FIRE_BALLS:
                if (timer.milliseconds() >= feedMs) {
                    // one ball completed
                    intake.setPower(0);
                    numberOfShots--;

                    if (numberOfShots <= 0) {
                        // done shooting
                        outtakeMotor.setVelocity(0);
                        enterState(State.SPIN_DOWN_ALL);
                    } else {
                        // fire next ball
                        timer.reset();
                        intake.setPower(1.0);
                    }
                }
                break;


            case SPIN_DOWN_ALL:
                if (timer.milliseconds() >= spinDownMs) {
                    enterState(State.IDLE);
                    busy = false;
                }
                break;
        }
    }


    /** Eject a number of balls (immediate fire with eject velocity). */
    public void eject(int balls) {
        if (busy) return;
        numberOfShots = Math.max(0, balls);
        busy = true;
        // set shooter to eject velocity and start firing immediately
        outtakeMotor.setVelocity(ejectVelocity);
        enterState(State.FIRE_BALLS);
        timer.reset();
        intake.setPower(1.0);
    }

    // Intake blockers
    public void blockIntake() {
        intakeBlockServo.setPosition(0.0); // use valid servo range; caller can adjust to correct values
    }
    public void unBlockIntake() {
        intakeBlockServo.setPosition(1.0);
    }

    // --- Manual angling ---
    public void manualAngling(double leftStickY) {
        double input = -leftStickY;
        if (Math.abs(input) < 0.08) return;
        anglePos += input * ANGLE_STEP;
        anglePos = Math.max(ANGLE_MIN, Math.min(ANGLE_MAX, anglePos));
        leftVerticalServo.setPosition(anglePos);
    }

    public void angleUp() {
        anglePos = Math.min(anglePos + ANGLE_STEP, ANGLE_MAX);
        leftVerticalServo.setPosition(anglePos);
    }

    public void angleDown() {
        anglePos = Math.max(anglePos - ANGLE_STEP, ANGLE_MIN);
        leftVerticalServo.setPosition(anglePos);
    }

    public void setAngle(double position) {
        anglePos = Math.max(ANGLE_MIN, Math.min(ANGLE_MAX, position));
        leftVerticalServo.setPosition(anglePos);
    }

    public double getAngle() { return anglePos; }

    // Intake/outtake direct controls (preserve existing API)
    public void startIntake(double power) {
        intake.setPower(power);
    }
    public void reverseIntake(double power) {
        intake.setPower(-power);
    }
    public void stopIntake() {
        intake.setPower(0);
    }

    public void startOuttake(double power) {
        outtakeMotor.setPower(power);
    }
    public void stopOuttake() {
        outtakeMotor.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- Shooter -----");
        telemetry.addData("Shooter Velocity", lastVelocity);
        telemetry.addData("State", state.toString());
        telemetry.addData("Remaining Shots", numberOfShots);
        telemetry.addData("Vertical Aim Pos", anglePos);
    }

    public boolean isBusy() { return busy; }
    public State getState() { return state; }
}
