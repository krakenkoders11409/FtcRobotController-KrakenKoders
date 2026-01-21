package org.firstinspires.ftc.teamcode.TestOrPrototype;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.hardware.Robot;
@TeleOp

public class OuttakePIDF extends OpMode {
    DcMotorEx flywheeleMotor;
    double highVelocity = 1300;
    double lowVelocity = 900;
    double currenttargetVelocity = highVelocity;
    double F = 0;
    double P = 0;

    double[] stepsSizes = {10, 1, 0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 1;

    @Override
    public void init() {

        flywheeleMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        flywheeleMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheeleMotor.setDirection(DcMotorSimple.Direction.REVERSE  );

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheeleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void loop() {
        // --- Set Gamepad commands ---
        if (gamepad2.aWasPressed()) {
            if (currenttargetVelocity == highVelocity) {
                currenttargetVelocity = lowVelocity;
            } else {
                currenttargetVelocity = highVelocity;
            }
        }
        if(gamepad2.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepsSizes.length;
        }
        if (gamepad2.dpadLeftWasPressed()) {
            F -= stepsSizes[stepIndex];
        }
        if (gamepad2.dpadRightWasPressed()) {
            F += stepsSizes[stepIndex];
        }
        if (gamepad2.dpadDownWasPressed()) {
            P -= stepsSizes[stepIndex];
        }
        if (gamepad2.dpadUpWasPressed()){
            P += stepsSizes[stepIndex];
        }

        // --- set new PIDF coefficients ---
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheeleMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        // Set velocity
        flywheeleMotor.setVelocity(currenttargetVelocity);

        double currentVelocity = flywheeleMotor.getVelocity();
        double error = currenttargetVelocity - currentVelocity;


        // Telemetry
        telemetry.addData("Target Veloctiy", currenttargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", currentVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-------------------------");
        telemetry.addData("Tuning P", "%.2f", P);
        telemetry.addData("Tuning F", "%.2f", F);
        telemetry.addData("Step size", "%.2f", stepsSizes[stepIndex]);
        telemetry.update();
    }

}
