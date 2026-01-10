import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name="testAuto_fixed")
public class testAuto extends LinearOpMode {
    private enum AutoStep { AIM, SHOOT, DRIVE, DONE }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // --- Set up wheels ---
        robot.drive.resetEncoders();
        robot.drive.setRunToPositionMode();

        // --- Set up vision / turret ---
        // Set turret TX offset to +10 degrees → right
        robot.turret.setTxOffset(-2);


        try {
            // if API has enableAutoAim(boolean)
            robot.turret.getClass().getMethod("enableAutoAim", boolean.class).invoke(robot.turret, true);
        } catch (Exception e) {
            // fallback to toggle if explicit method not present
            robot.turret.toggleAutoAim();
        }

        // Whitelist Tag 24 (Red)
        robot.vision.clearAllowedTags();
        robot.vision.addAllowedTag(24);

        // Timer for sequential actions and timeouts
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        // --- IMPORTANT: initialize autoStep ---
        AutoStep autoStep = AutoStep.AIM;

        // safety timeout values (adjust as needed)
        final double SHOOT_TIMEOUT_S = 3.0;    // max seconds to wait for shooter
        final double AIM_TOLERANCE_DEG = 1.5;  // aim tolerance in degrees

        while (opModeIsActive() && autoStep != AutoStep.DONE) {
            // Update subsystems
            robot.vision.update();
            double tx = robot.vision.getTx();            // may be undefined if no target
            boolean hasTarget = robot.vision.hasTarget();

            // Auto-aim turret - pass hasTarget to avoid NaNs inside turret update
            robot.turret.update(0, tx, hasTarget);

            // Shooter update
            robot.shooter.update();

            // Drive update
            robot.drive.update();

            // Telemetry for debugging
            telemetry.addData("Step", autoStep);
            telemetry.addData("HasTarget", hasTarget);
            telemetry.addData("Tx", hasTarget ? String.format("%.2f", tx) : "N/A");
            telemetry.addData("TxOffset", robot.turret.getTxOffset());
            telemetry.addData("ShooterBusy", robot.shooter.isBusy());
            telemetry.addData("DriveBusy", robot.drive.isBusy());

            switch (autoStep) {
                case AIM:
                    // Only consider aiming if we have a target
                    //robot.turret.turretRunToPosition();
                    break;

                case SHOOT:
                    // Proceed when shooter finished OR when timeout elapsed
                    if (!robot.shooter.isBusy()) {
                        // advance to driving
                        autoStep = AutoStep.DRIVE;
                        robot.drive.resetEncoders();
                        robot.drive.setRunToPositionMode();
                        robot.drive.setTargetForwardInches(10, 0.8);
                    } else if (timer.seconds() > SHOOT_TIMEOUT_S) {
                        // shooter stuck — bail out to next step to avoid stall
                        telemetry.addData("WARN", "Shooter timeout, continuing.");
                        autoStep = AutoStep.DRIVE;
                        robot.drive.resetEncoders();
                        robot.drive.setRunToPositionMode();
                        robot.drive.setTargetForwardInches(10, 0.8);
                    }
                    break;

                case DRIVE:
                    if (!robot.drive.isBusy()) {
                        autoStep = AutoStep.DONE;
                    }
                    break;

                case DONE:
                    // Optionally stop motors or set a safe state
                    break;
            }

            telemetry.update();
            // allow other threads / hardware to run
            idle();
        }

        // cleanup / final telemetry
        telemetry.addData("Auto", "Finished");
        telemetry.update();
    }
}
