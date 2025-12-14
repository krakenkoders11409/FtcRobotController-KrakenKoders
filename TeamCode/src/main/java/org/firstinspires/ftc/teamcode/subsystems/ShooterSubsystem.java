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
        SERVO_UP, // Servo goes to position 0.5
        LIFT_HOLD, // Servo waits
        SERVO_DOWN, // Servo goes to position 0.8
        SPIN_DOWN,
        SPIN_UP_EJECT
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
    private final int velocityTolerance = 35; // how far away from the target velocity is OK
    private String shotType = "";

    private final ElapsedTime timer = new ElapsedTime();
    private State state = State.IDLE;
    private boolean busy = false;


    private int numberOfShots = 0;
    private int numberOfBalls = 0;
    private double lastVelocity = 0;

    // Lift Tunables
    private static final double LIFT_UP_POS = 0.5;
    private static final double LIFT_DOWN_POS = 0.8;


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
            return; // ignore if already running
        }
        numberOfShots = shots;
        shotType = type;
        busy = true;
        state = State.SPIN_UP;
        timer.reset();

        if (Objects.equals(shotType, "short")){
            outtakeMotor.setVelocity(shortShotVelocity);
        } else {
            outtakeMotor.setVelocity(longShotVelocity);
        }
        intake.setPower(1);
    }

    /** Call this every loop to advance the sequence without blocking. */
    public void update() {
        switch (state) {

            case IDLE:
                break;

            case SPIN_UP:
                lastVelocity = outtakeMotor.getVelocity();
                int targetVel = shotType.equals("short") ? shortShotVelocity : longShotVelocity;

                if (Math.abs(targetVel - lastVelocity) < velocityTolerance) {
                    intake.setPower(0);

                    intakeArmServo.setPosition(LIFT_UP_POS);   // start lift
                    timer.reset();
                    state = State.SERVO_UP;
                }
                break;

            case SERVO_UP:
                // give servo one loop to move
                if (timer.milliseconds() > 250) {
                    intakeArmServo.setPosition(LIFT_DOWN_POS);   // lower arm
                    timer.reset();
                    state = State.SERVO_DOWN;
                }
                break;

            case SERVO_DOWN:
                // no delay needed here
                if (timer.milliseconds() > 250) {
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
                } else {

                    outtakeMotor.setPower(0);
                    state = State.IDLE;
                    busy = false;
                }
                break;

            case SPIN_UP_EJECT:
                lastVelocity = outtakeMotor.getVelocity();
                if (Math.abs(ejectVelocity - lastVelocity) < velocityTolerance) {
                    intake.setPower(intakePower);
                    intakeArmServo.setPosition(0);
                    timer.reset();
                    state = State.SERVO_UP;
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
        state = State.SPIN_UP_EJECT;
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
            state = state.SERVO_UP;

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
    
