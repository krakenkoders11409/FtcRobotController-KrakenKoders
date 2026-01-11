package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class ShooterSubsystem {
    public enum State { IDLE, SPIN_UP, FEED, SPIN_DOWN, EJECT }

    // Setup for Angling Servos -----------------------------------------------------------
    private static final double ANGLE_MIN = 0.35;
    private static final double ANGLE_MAX = 1;
    private static final double ANGLE_STEP = 0.02;
    private double anglePos = 0.5; // start centered




    private final DcMotor intake;
    private final Servo intakeBlockServo;
    private final DcMotorEx outtakeMotor;

    private final Servo leftVerticalServo;

    // Tunables
    private final int shortShotVelocity = 950; // spin power
    private final int longShotVelocity = 1250; // spin power
    private final int AutoShortShotVelocity = 960; // spin power
    private final int AutoLongShotVelocity = 1000; // spin power

    private final int ejectVelocity = 500;
    private final double targetPower = 1.0; // spin power
    private final double intakePower = 1.0; // spin power
    private final long spinUpMs = 2000; // wait time before feed
    private final long feedMs = 2000; // time to press the ball
    private final long spinDownMs = 1000; // optional coast-down window
    private final int velocityTolerance = 35; // how far away from the target velocity is OK
    private String shotType = "";

    private final ElapsedTime timer = new ElapsedTime();
    private State state = State.IDLE;
    private boolean busy = false;
    private boolean toggleState = false;
    private boolean previousTrigger = false;


    private int numberOfShots = 0;
    private int numberOfBalls = 0;
    private double lastVelocity = 0;

    // --- Dynamic Shot Power ---
//    public static double shotSpeed(double distance){
//        return MathFunctions
//    }
//
//    public static double hoodAngle(double distance){
//        return MathFunctions
//    }


    public ShooterSubsystem(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeBlockServo = hardwareMap.get(Servo.class, "intakeBlockServo");
        leftVerticalServo = hardwareMap.get(Servo.class, "leftVerticalServo");



        // Encoder logic -------------------------------------------------
        outtakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set motor directions (adjust if movement is inverted) ----------
        outtakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intakeBlockServo.setDirection(Servo.Direction.REVERSE);
        leftVerticalServo.setDirection(Servo.Direction.REVERSE);



        // Set motor behavior ----------------------------------------------
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /** Start a single, timed shot. Returns immediately (non-blocking). */
    public void startShot(int shots, String type) {
        if (busy) {
            //telemetry.addLine("Shooter is busy");
            //telemetry.update();
            return; // ignore if already running
        }
        numberOfShots = shots;
        shotType = type;
        //telemetry.addLine("Shot Initiated");
        //telemetry.update();
        busy = true;
        state = State.SPIN_UP;
        timer.reset();

        //launcher.setPower(targetPower);
        if (Objects.equals(shotType, "short")) {
            outtakeMotor.setVelocity(shortShotVelocity);
        } else if (Objects.equals(shotType, "long")) {
            outtakeMotor.setVelocity(longShotVelocity);
        } else if (Objects.equals(shotType, "autoShort")) {
            outtakeMotor.setVelocity(AutoShortShotVelocity);
        } else {
            if (Objects.equals(shotType, "autoLong")) {
                outtakeMotor.setVelocity(AutoLongShotVelocity);
            }
        }
    }

    /** Call this every loop to advance the sequence without blocking. */
    public void update() {
        switch (state) {
            case IDLE:
                // nothing
                break;

            case SPIN_UP:
                lastVelocity = outtakeMotor.getVelocity();
                if ("short".equals(shotType)) {
                    if (Math.abs(shortShotVelocity - outtakeMotor.getVelocity()) < velocityTolerance) {
                        //if (timer.milliseconds() >= spinUpMs) {
                        state = State.FEED;
                        timer.reset();
                        unBlockIntake();
                        intake.setPower(intakePower); // push ball into shooter
                    }
                } else {
                    if (Math.abs(longShotVelocity - outtakeMotor.getVelocity()) < velocityTolerance) {
                        //if (timer.milliseconds() >= spinUpMs) {
                        state = State.FEED;
                        timer.reset();
                        blockIntake();
                        intake.setPower(intakePower); // push ball into shooter

                    }
                }

                break;

            case EJECT:
                lastVelocity=  outtakeMotor.getVelocity();
                if (Math.abs(ejectVelocity - outtakeMotor.getVelocity()) < velocityTolerance) {
                    // if (timer.milliseconds() >= spinUpMs) {
                    state = State.FEED;
                    timer.reset();
                    intake.setPower(intakePower); // push ball into shooter

                }
                break;

            case FEED:
                if (timer.milliseconds() >= feedMs) {
                    state = State.SPIN_DOWN;
                    timer.reset();
                    outtakeMotor.setVelocity(0);
                    intake.setPower(0);
                }
                break;

            case SPIN_DOWN:
                if (numberOfShots > 1){
                    state = State.SPIN_UP;
                    if (shotType == "short") {
                        outtakeMotor.setVelocity(shortShotVelocity);
                    } else {
                        outtakeMotor.setVelocity(longShotVelocity);
                    }
                    numberOfShots--;
                    //telemetry.addLine("Shots left:" + numberOfShots);
                }
                if (timer.milliseconds() >= spinDownMs) {
                    state = State.IDLE;
                    busy = false;
                }
                break;
        }
    }

    public void eject(int balls) {
        if(busy) {
            //telemetry.addLine("Shooter is busy");
            return; // ignore if already running
        }
        numberOfBalls = balls;
        //telemetry.addLine("Ejection Initiated");
        busy = true;
        state = State.EJECT;
        timer.reset();

        // launcher.setPower(targetPower);
        outtakeMotor.setVelocity(ejectVelocity);
    }
    public void blockIntake() {
        intakeBlockServo.setPosition(-0.5);
    }
    public void unBlockIntake() {
        intakeBlockServo.setPosition(0.5);
    }


    // --- Manual Angling ---
    public void manualAngling(double leftStickY) {

        double input = -leftStickY;

        // Deadzone to prevent jitter
        if (Math.abs(input) < 0.08) {
            return; // hold position
        }

        anglePos += input * ANGLE_STEP;

        // Clamp to limits
        anglePos = Math.max(ANGLE_MIN, Math.min(ANGLE_MAX, anglePos));

        leftVerticalServo.setPosition(anglePos);
    }
    public void hoodAngle(double position) {
        leftVerticalServo.setPosition(position);
    }


    public void angleUp() {
        anglePos += ANGLE_STEP;
        anglePos = Math.min(anglePos, ANGLE_MAX);
        leftVerticalServo.setPosition(anglePos);
    }

    public void angleDown() {
        anglePos -= ANGLE_STEP;
        anglePos = Math.max(anglePos, ANGLE_MIN);
        leftVerticalServo.setPosition(anglePos);
    }

    public void setAngle(double position) {
        anglePos = Math.max(ANGLE_MIN, Math.min(ANGLE_MAX, position));
        leftVerticalServo.setPosition(anglePos);
    }

    public double getAngle() {
        return anglePos;
    }




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

    // --- Getters ---
    public double getOuttakeVelocity() {
        return outtakeMotor.getVelocity();
    }


    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- Shooter -----");
        telemetry.addData("Shooter Velocity = ", getOuttakeVelocity());
        telemetry.addData("Vertical Aim Pos", anglePos);

    }

    public boolean isBusy() { return busy; }
    public State getState() { return state; }


}