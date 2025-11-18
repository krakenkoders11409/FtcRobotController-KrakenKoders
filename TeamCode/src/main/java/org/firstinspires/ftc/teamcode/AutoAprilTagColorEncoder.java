package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Auto: AprilTag + Color (Encoders)")
public class AutoAprilTagColorEncoder extends LinearOpMode {

    // --- Motors ---
    private DcMotor bl0, br1, fl2, fr3, intake, intake2;
    private Servo servo;

    // --- Vision ---
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private PredominantColorProcessor colorProcessor;

    // --- Encoder constants ---
    static final double COUNTS_PER_MOTOR_REV = 537.7; // GoBilda 5202/5203 motors
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77953; // 96mm GoBilda wheels
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() throws InterruptedException {

        // --- Hardware Setup ---
        bl0 = hardwareMap.get(DcMotor.class, "bl0");
        br1 = hardwareMap.get(DcMotor.class, "br1");
        fl2 = hardwareMap.get(DcMotor.class, "fl2");
        fr3 = hardwareMap.get(DcMotor.class, "fr3");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        servo = hardwareMap.get(Servo.class, "servo");

        fl2.setDirection(DcMotorSimple.Direction.REVERSE);
        bl0.setDirection(DcMotorSimple.Direction.REVERSE);
        fr3.setDirection(DcMotorSimple.Direction.FORWARD);
        br1.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{fl2, fr3, bl0, br1}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // --- Vision Setup ---
        aprilTag = new AprilTagProcessor.Builder()
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        colorProcessor = new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.25, 0.25, 0.25, -0.25))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE
                )
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .addProcessor(colorProcessor)
                .setCameraResolution(new Size(320, 240))
                .build();

        telemetry.addLine("Initializing Vision...");
        telemetry.update();

        waitForStart();

        // --- Step 1: Detect AprilTag ---
        int tagId = -1;
        if (!aprilTag.getDetections().isEmpty()) {
            tagId = aprilTag.getDetections().get(0).id;
            telemetry.addData("AprilTag Detected:", tagId);
        } else {
            telemetry.addLine("No AprilTag found, defaulting to Zone 1");
        }
        telemetry.update();

        // --- Step 2: Find Ball by Color ---
        boolean ballFound = false;
        while (opModeIsActive() && !ballFound) {
            PredominantColorProcessor.Result result = colorProcessor.getAnalysis();
            String colorName = result.closestSwatch.toString();

            telemetry.addData("Detected Color:", colorName);
            telemetry.update();

            if (colorName.contains("GREEN") || colorName.contains("PURPLE")) {
                telemetry.addLine("Ball detected! Approaching...");
                telemetry.update();
                ballFound = true;
                encoderDrive(0.4, 6); // move forward 6 inches toward ball
            } else {
                encoderDrive(0.3, 3); // small forward scan
            }
        }

        // --- Step 3: Collect Ball ---
        if (ballFound) {
            intake.setPower(1);
            intake2.setPower(1);
            sleep(1200);
            intake.setPower(0);
            intake2.setPower(0);
        }

        // --- Step 4: Move to Zone based on AprilTag ---
        if (tagId == 1) {
            strafeLeft(0.5, 10);  // Zone 1
        } else if (tagId == 2) {
            encoderDrive(0.5, 12); // Zone 2
        } else if (tagId == 3) {
            strafeRight(0.5, 10); // Zone 3
        } else {
            encoderDrive(0.5, 8); // Default
        }

        telemetry.addLine("✅ Autonomous Complete");
        telemetry.update();
    }

    // --- Encoder Drive (forward/backward) ---
    private void encoderDrive(double power, double inches) {
        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        for (DcMotor m : new DcMotor[]{fl2, fr3, bl0, br1}) {
            m.setTargetPosition(m.getCurrentPosition() + moveCounts);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (opModeIsActive() &&
                (fl2.isBusy() || fr3.isBusy() || bl0.isBusy() || br1.isBusy())) {
            telemetry.addData("Moving", "%d counts", moveCounts);
            telemetry.update();
        }

        stopDrive();
        for (DcMotor m : new DcMotor[]{fl2, fr3, bl0, br1}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // --- Encoder Strafe Left ---
    private void strafeLeft(double power, double inches) {
        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        fl2.setTargetPosition(fl2.getCurrentPosition() - moveCounts);
        bl0.setTargetPosition(bl0.getCurrentPosition() + moveCounts);
        fr3.setTargetPosition(fr3.getCurrentPosition() + moveCounts);
        br1.setTargetPosition(br1.getCurrentPosition() - moveCounts);

        for (DcMotor m : new DcMotor[]{fl2, fr3, bl0, br1}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (opModeIsActive() &&
                (fl2.isBusy() || fr3.isBusy() || bl0.isBusy() || br1.isBusy())) {
            telemetry.addLine("Strafing Left...");
            telemetry.update();
        }

        stopDrive();
        for (DcMotor m : new DcMotor[]{fl2, fr3, bl0, br1}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // --- Encoder Strafe Right ---
    private void strafeRight(double power, double inches) {
        int moveCounts = (int) (inches * COUNTS_PER_INCH);

        fl2.setTargetPosition(fl2.getCurrentPosition() + moveCounts);
        bl0.setTargetPosition(bl0.getCurrentPosition() - moveCounts);
        fr3.setTargetPosition(fr3.getCurrentPosition() - moveCounts);
        br1.setTargetPosition(br1.getCurrentPosition() + moveCounts);

        for (DcMotor m : new DcMotor[]{fl2, fr3, bl0, br1}) {
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(power);
        }

        while (opModeIsActive() &&
                (fl2.isBusy() || fr3.isBusy() || bl0.isBusy() || br1.isBusy())) {
            telemetry.addLine("Strafing Right...");
            telemetry.update();
        }

        stopDrive();
        for (DcMotor m : new DcMotor[]{fl2, fr3, bl0, br1}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void stopDrive() {
        fl2.setPower(0);
        fr3.setPower(0);
        bl0.setPower(0);
        br1.setPower(0);
    }
}
