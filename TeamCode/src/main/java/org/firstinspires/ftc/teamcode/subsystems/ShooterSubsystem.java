package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;

public class ShooterSubsystem {
    public enum State {
        IDLE,
        SPIN_UP,

        LIFT_UP, // Servo goes to position 0.5
        LIFT_HOLD, // Servo waits
        LIFT_DOWN, // Servo goes to position 0.8

        FEED,
        SPIN_DOWN,
        EJECT
    }

    // Setup for Angling Servos -----------------------------------------------------------
    private final double angleMax = 1;
    private final double angleMin = 0;
    private double anglePos = 0;
    // Setup for Angling Speed
    private final double angSpeed = 0.8;


    private final DcMotor intake;
    private final Servo intakeArmServo;
    private final DcMotorEx outtakeMotor;

    private final CRServo  leftVerticalServo, rightVerticalServo;

    // Tunables
    private final int shortShotVelocity = 1400; // spin power
    private final int longShotVelocity = 2000; // spin power
    private final int ejectVelocity = 1000;
    private final double targetPower = 1.0; // spin power
    private final double intakePower = 1.0; // spin power
    private final long spinUpMs = 2000; // wait time before feed
    private final long feedMs = 1000; // time to press the ball
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

    // Lift Tunables
    private static final double LIFT_UP_POS = 0.5;
    private static final double LIFT_DOWN_POS = 0.8;
    private static final long LIFT_HOLD_MS = 900;


    public ShooterSubsystem(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
        leftVerticalServo = hardwareMap.get(CRServo.class, "leftVerticalServo");
        rightVerticalServo = hardwareMap.get(CRServo.class, "rightVerticalServo");



        // Encoder logic -------------------------------------------------
        outtakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftVerticalServo.setDirection(CRServo.Direction.REVERSE);
        rightVerticalServo.setDirection(CRServo.Direction.REVERSE);

        // Set motor directions (adjust if movement is inverted) ----------
        outtakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.REVERSE);
        intakeArmServo.setDirection(Servo.Direction.REVERSE);


        // Set motor behavior ----------------------------------------------
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /** Start a single, timed shot. Returns immediately (non-blocking). */
    public void startShot(int shots, String type) {
        if(busy) {
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
        if (Objects.equals(shotType, "short")){
            outtakeMotor.setVelocity(shortShotVelocity);
        } else {
            outtakeMotor.setVelocity(longShotVelocity);
        }
    }

    /** Call this every loop to advance the sequence without blocking. */
    public void update() {
        switch (state) {

            case IDLE:
                break;

            case SPIN_UP:
                lastVelocity = outtakeMotor.getVelocity();

                int targetVel = shotType.equals("short")
                        ? shortShotVelocity
                        : longShotVelocity;

                if (Math.abs(targetVel - lastVelocity) < velocityTolerance) {
                    timer.reset();
                    intakeArmServo.setPosition(0.5);   // start lift
                    state = State.LIFT_UP;
                }
                break;

            case LIFT_UP:
                // give servo one loop to move
                timer.reset();
                state = State.LIFT_HOLD;
                break;

            case LIFT_HOLD:
                if (timer.milliseconds() >= 900) {
                    intakeArmServo.setPosition(0.8);   // lower arm
                    state = State.LIFT_DOWN;
                }
                break;

            case LIFT_DOWN:
                // no delay needed here
                timer.reset();
                intake.setPower(intakePower);
                state = State.FEED;
                break;

            case FEED:
                if (timer.milliseconds() >= feedMs) {
                    intake.setPower(0);
                    outtakeMotor.setVelocity(0);
                    timer.reset();
                    state = State.SPIN_DOWN;
                }
                break;

            case SPIN_DOWN:
                if (numberOfShots > 1) {
                    numberOfShots--;
                    timer.reset();
                    outtakeMotor.setVelocity(
                            shotType.equals("short")
                                    ? shortShotVelocity
                                    : longShotVelocity
                    );
                    state = State.SPIN_UP;
                } else if (timer.milliseconds() >= spinDownMs) {
                    state = State.IDLE;
                    busy = false;
                }
                break;

            case EJECT:
                lastVelocity = outtakeMotor.getVelocity();
                if (Math.abs(ejectVelocity - lastVelocity) < velocityTolerance) {
                    intake.setPower(intakePower);
                    intakeArmServo.setPosition(0);
                    timer.reset();
                    state = State.FEED;
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

    public void manualAngling( double vertical){
        double anglePower = vertical * angSpeed;
        leftVerticalServo.setPower(anglePower);
        rightVerticalServo.setPower(-anglePower);
    }

    public void liftBall(){
        if (state != State.LIFT_HOLD) return;{
            intakeArmServo.setPosition(LIFT_UP_POS);
            timer.reset();
            state = state.LIFT_UP;

        }
    }
    public void startIntake() {
        intake.setPower(1);
    }

    public void stopIntake() {
        intake.setPower(0);
    }



    public void startOuttake() {
        outtakeMotor.setPower(1);
    }

    public void stopOuttake() {
        outtakeMotor.setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- Shooter -----");
        telemetry.addData("Shooter Velocity = ", lastVelocity);
        telemetry.addData("Servo Position = ", intakeArmServo.getPosition());
        telemetry.addData("Shooter State = ", state);
    }

    public boolean isBusy() { return busy; }
    public State getState() { return state; }


}
    
