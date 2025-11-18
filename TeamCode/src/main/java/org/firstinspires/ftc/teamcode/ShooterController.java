package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ShooterController {
    public enum State { IDLE, SPIN_UP, FEED, SPIN_DOWN, EJECT }

    private final DcMotor intake, intake2;
    private final DcMotorEx launcher;
    private final LinearOpMode opMode;

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
    private int numberOfShots = 0;
    private int numberOfBalls = 0;

    public ShooterController(DcMotorEx passedInLauncherMoter, DcMotor passedIntakeMotor, DcMotor passedIntakeMotor2, LinearOpMode opMode) {
        this.launcher = passedInLauncherMoter;
        this.intake = passedIntakeMotor;
        this.intake2 = passedIntakeMotor2;
        this.opMode = opMode;
    }

    /** Start a single, timed shot. Returns immediately (non-blocking). */
    public void startShot(int shots, String type) {
        if(busy) { 
            opMode.telemetry.addLine("Shooter is busy");
            opMode.telemetry.update();
            return; // ignore if already running
        }
        numberOfShots = shots;
        shotType = type;
        opMode.telemetry.addLine("Shot Initiated");
        opMode.telemetry.update();
        busy = true;
        state = State.SPIN_UP;
        timer.reset();

        //launcher.setPower(targetPower);
        if (shotType == "short"){
            launcher.setVelocity(shortShotVelocity);
        } else {
            launcher.setVelocity(longShotVelocity);
        }
    }

    /** Call this every loop to advance the sequence without blocking. */
    public void update() {
        switch (state) {
            case IDLE:
                // nothing
                break;

            case SPIN_UP:
                opMode.telemetry.addLine("Velocity= " + launcher.getVelocity());
                opMode.telemetry.update();
                if (shotType == "short"){
                    if (Math.abs(shortShotVelocity - launcher.getVelocity()) < velocityTolerance) {
                    //if (timer.milliseconds() >= spinUpMs) {
                        state = State.FEED;
                        timer.reset();
                        intake.setPower(intakePower); // push ball into shooter
                        intake2.setPower(0);
                    }
                } else {
                    if (Math.abs(longShotVelocity - launcher.getVelocity()) < velocityTolerance) {
                    //if (timer.milliseconds() >= spinUpMs) {
                        state = State.FEED;
                        timer.reset();
                        intake.setPower(intakePower); // push ball into shooter
                        intake2.setPower(0);
                    }
                }
                
                break;

            case EJECT:
                opMode.telemetry.addLine("Velocity= " + launcher.getVelocity());
                opMode.telemetry.update();
                if (Math.abs(ejectVelocity - launcher.getVelocity()) < velocityTolerance) {
                // if (timer.milliseconds() >= spinUpMs) {
                    state = State.FEED;
                    timer.reset();
                    intake.setPower(intakePower); // push ball into shooter
                    intake2.setPower(0);
                }
                break;

            case FEED:
                if (timer.milliseconds() >= feedMs) {
                    state = State.SPIN_DOWN;
                    timer.reset();
                    launcher.setVelocity(0);
                    intake.setPower(0);
                }
                break;

            case SPIN_DOWN:
                if (numberOfShots > 1){
                    state = State.SPIN_UP;
                    if (shotType == "short") {
                    launcher.setVelocity(shortShotVelocity);
                    } else {
                        launcher.setVelocity(longShotVelocity);
                    }
                    intake2.setPower(intakePower);
                    numberOfShots--;
                    opMode.telemetry.addLine("Shots left:" + numberOfShots);
                    opMode.telemetry.update();
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
            opMode.telemetry.addLine("Shooter is busy");
            opMode.telemetry.update();
            return; // ignore if already running
        }
        numberOfBalls = balls;
        opMode.telemetry.addLine("Ejection Initiated");
        opMode.telemetry.update();
        busy = true;
        state = State.EJECT;
        timer.reset();

        // launcher.setPower(targetPower);
        launcher.setVelocity(ejectVelocity);
    }



    public boolean isBusy() { return busy; }
    public State getState() { return state; }
}
    
