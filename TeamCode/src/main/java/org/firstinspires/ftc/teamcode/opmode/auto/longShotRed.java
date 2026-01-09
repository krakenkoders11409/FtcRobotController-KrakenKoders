import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous
public class longShotRed extends LinearOpMode {

    // State machine for autonomous steps
    private enum Step { AIM, SHOOT, DRIVE, DONE }

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);

        // Set turret TX offset to +10 degrees → right
        robot.turret.setTxOffset(10);

        // Enable auto aim
        robot.turret.toggleAutoAim();
        robot.turret.enableAutoAim();
        robot.vision.clearAllowedTags();
        robot.vision.addAllowedTag(24);


        waitForStart();

        // Timer for sequential actions
        ElapsedTime timer = new ElapsedTime();

        Step step = Step.AIM;
        timer.reset();

        while (opModeIsActive() && step != Step.DONE) {
            // Update vision for auto-aim
            robot.vision.update();
            double tx = robot.vision.getTx();
            boolean hasTarget = robot.vision.hasTarget();

            // Auto-aim turret
            robot.turret.update(0, tx, hasTarget);

            // Shooter update
            robot.shooter.update();

            // Drive update
            robot.drive.update();

            switch (step) {
                case AIM:
                    if (Math.abs(tx + robot.turret.getTxOffset()) < 1.5) {
                        step = Step.SHOOT;
                        robot.shooter.startShot(1, "long");
                        timer.reset();
                    }
                    break;

                case SHOOT:
                    if (!robot.shooter.isBusy()) {
                        step = Step.DRIVE;
                        robot.drive.resetEncoders();
                        robot.drive.setRunToPositionMode();
                        robot.drive.setTargetForwardInches(10, 0.8);
                    }
                    break;

                case DRIVE:
                    if (!robot.drive.isBusy()) {
                        step = Step.DONE;
                    }
                    break;

                case DONE:
                    break;
            }

            telemetry.update();
        }
    }
}
