package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import static org.firstinspires.ftc.teamcode.constants.DriveConstants.DriveConstants.TICKS_PER_INCH;


public class DriveSubsystem {

    private final DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;


    // Odometry Wheels Setup
    private final DcMotor odoForwardWheel;
    private final DcMotor odoStrafeWheel;

    // IMU
    private IMU imu;

    // Field coordinates
    private double xPos = 0.0;
    private double yPos = 0.0;
    private double heading = 0.0;


    // Previous Encoder Position
    private int lastForwardEncoder = 0;
    private int lastStrafeEncoder = 0;

    private double speedMultiplier = 1.0;
    private static final double TICKS_PER_DEGREE = 7.75;
    private static final double ODO_TICKS_PER_INCH = 8192.0 / (Math.PI * 2.0);
    double lastLfPower = 0;
    double lastLbPower = 0;
    double lastRfPower = 0;
    double lastRbPower = 0;

    public DriveSubsystem(HardwareMap hardwareMap) {
        // Initialize motors from hardware map ------------------------
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        odoForwardWheel = hardwareMap.get(DcMotor.class, "odoForwardWheel");
        odoStrafeWheel = hardwareMap.get(DcMotor.class, "odoStrafeWheel");

        // Set motor directions (adjust if movement is inverted) ----------
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set motor behavior ----------------------------------------------
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Ensure encoders are enabled ----------------------------------------------
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        odoForwardWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odoStrafeWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        odoForwardWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        odoStrafeWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));
    }


    public void updateOdometry() {
        heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        int forwardTicks = odoForwardWheel.getCurrentPosition();
        int strafeTicks = odoStrafeWheel.getCurrentPosition();

        int deltaForwardTicks = forwardTicks - lastForwardEncoder;
        int deltaStrafeTicks = strafeTicks - lastStrafeEncoder;

        lastForwardEncoder = forwardTicks;
        lastStrafeEncoder = strafeTicks;

        double deltaForwardInches = deltaForwardTicks / ODO_TICKS_PER_INCH;
        double deltaStrafeInches = deltaStrafeTicks / ODO_TICKS_PER_INCH;

        double fieldDeltaX = deltaStrafeInches * Math.cos(heading) - deltaForwardInches * Math.sin(heading);
        double fieldDeltaY = deltaStrafeInches * Math.sin(heading) - deltaForwardInches * Math.cos(heading);

        xPos += fieldDeltaX;
        yPos += fieldDeltaY;
    }


    // Field Oriented Drive -----------------------------------------------------------------------
    public void driveFieldOriented(double forward, double strafe, double turn){
        double rotStrafe = strafe * Math.cos(heading) - forward * Math.sin(heading);
        double rotForward = strafe * Math.sin(heading) + forward * Math.cos(heading);

        drive(rotForward, rotStrafe, turn);
    }







// Normal Drive ---------------------------------------------------------------------------------
    public void drive(double forward, double strafe, double turn) {

        strafe *= 1.1; // minor correction for imperfect strafing

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);

        double lfPower = (forward + strafe + turn) / denominator * speedMultiplier;
        double lbPower = (forward - strafe + turn) / denominator * speedMultiplier;
        double rfPower = (forward - strafe - turn) / denominator * speedMultiplier;
        double rbPower = (forward + strafe - turn) / denominator * speedMultiplier;

        lastLbPower = lbPower;
        lastLfPower = lfPower;
        lastRbPower = rbPower;
        lastRfPower = rfPower;

        frontLeftMotor.setPower(lfPower);
        frontRightMotor.setPower(rfPower);
        backLeftMotor.setPower(lbPower);
        backRightMotor.setPower(rbPower);
    }

    // Slow Mode ----------------------------------------------------------------------------------
    public void toggleSlowMode() {
        speedMultiplier = (speedMultiplier == 1.0) ? 0.5 : 1.0;
    }

    // Stop All Motors ---------------------------------------------------------------------------
    public void stop() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
    public boolean isBusy() {
        return frontLeftMotor.isBusy() || backLeftMotor.isBusy() ||
                frontRightMotor.isBusy() || backRightMotor.isBusy();
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- Motors -----");
        telemetry.addData("FL Power", lastLfPower);
        telemetry.addData("FR Power", (-1 *lastRfPower));
        telemetry.addData("BL Power", lastLbPower);
        telemetry.addData("BR Power", lastRbPower);

        telemetry.addData("X:", xPos);
        telemetry.addData("Y:", yPos);
        telemetry.addData("Heading:", Math.toDegrees((heading)));
    }

    public void update() {
        // Add odometry later if needed
    }

//     ------------------------ AUTONOMOUS HELPERS ---------------------------------------------------

    public void resetEncoders() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRunToPositionMode() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetDrive(double forwardInches, double strafeInches, double rotateDegrees, double power) {
        if (power > 0) {
            frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        } else {
            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
            backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        }


        resetEncoders();
        int forwardTicks = (int) Math.round(forwardInches * TICKS_PER_INCH);
        int strafeTicks  = (int) Math.round(strafeInches  * TICKS_PER_INCH);
        int rotateTicks  = (int) Math.round(rotateDegrees * TICKS_PER_DEGREE);

        // Mecanum kinematics
        int fl = forwardTicks + strafeTicks + rotateTicks;
        int fr = forwardTicks - strafeTicks - rotateTicks;
        int bl = forwardTicks - strafeTicks + rotateTicks;
        int br = forwardTicks + strafeTicks - rotateTicks;

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + fl);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + fr);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + bl);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + br);

        double p = Math.abs(power);  // must be > 0
        frontLeftMotor.setPower(p);
        frontRightMotor.setPower(p);
        backLeftMotor.setPower(p);
        backRightMotor.setPower(p);

    }
}
