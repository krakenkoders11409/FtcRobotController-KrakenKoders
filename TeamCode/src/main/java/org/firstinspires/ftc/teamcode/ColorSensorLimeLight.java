//package org.firstinspires.ftc.teamcode;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//
//public class ColorSensorLimeLight {
//    Limelight3A limelight;
//    public void init(){
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(100);
//        limelight.start();
//
//        limelight.pipelineSwitch(0);
//        result.getPipelineIndex();
//
//        LLResult result = limelight.getLatestResult();
//        if (result != null && result.isValid()) {
//            double tx = result.getTx(); // How far left or right the target is (degrees)
//            double ty = result.getTy(); // How far up or down the target is (degrees)
//            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
//
//            telemetry.addData("Target X", tx);
//            telemetry.addData("Target Y", ty);
//            telemetry.addData("Target Area", ta);
//        } else {
//            telemetry.addData("Limelight", "No Targets");
//        }
//
//
//        // Sending numbers to Python
//        double[] inputs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
//        limelight.updatePythonInputs(inputs);
//
//// Getting numbers from Python
//        double[] pythonOutputs = result.getPythonOutput();
//        if (pythonOutputs != null && pythonOutputs.length > 0) {
//            double firstOutput = pythonOutputs[0];
//            telemetry.addData("Python output:", firstOutput);
//        }
//    }
//}