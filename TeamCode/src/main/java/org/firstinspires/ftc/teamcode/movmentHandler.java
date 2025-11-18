package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class movementHandler {
    // ---------------------------------------------------------------------------------------------
    // Motor Set-Up
    // ---------------------------------------------------------------------------------------------

    // Define Motors
    private DcMotor bl0, br1, fl2, fr3;

    // Speed Modifier
    private double diveScale = 1.0;

    public movementHandler(HardwareMap hardwareMap) {
        // Init Motors
        fl2 = hardwareMap.get(DcMotor.class, "fl2");
        fr3 = hardwareMap.get(DcMotor.class, "fr3");
        bl0 = hardwareMap.get(DcMotor.class, "bl0");
        br1 = hardwareMap.get(DcMotor.class, "br1");

        // Set Motor Direction
        fl2.setDirection(DcMotor.Direction.REVERSE);
        bl0.setDirection(DcMotor.Direction.REVERSE);

        fr3.setDirection(DcMotor.Direction.FORWARD);
        br1.setDirection(DcMotor.Direction.FORWARD);

        // No Power Break Behavior
        fl2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // ---------------------------------------------------------------------------------------------
    // TELEOP MOVEMENT
    // ---------------------------------------------------------------------------------------------

    public void driveFieldOriented(double y, double x, double rx) {
        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        normalizeAndSet(flPower, blPower, frPower, brPower);
    }

    // Allow Tele-Op Speed Adjustment
    public void setDriveScale(double scale) {
        driveScale = scale;
    }

    // ---------------------------------------------------------------------------------------------
    // AUTONOMOUS HELPER
    // ---------------------------------------------------------------------------------------------

    public void drive(double flPower, double blPower, double frPower, double brPower) {

    }



}
