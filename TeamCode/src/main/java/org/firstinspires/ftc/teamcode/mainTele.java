package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import org.firstinspires.ftc.teamcode.SimpleLedController;

// @Disabled
@TeleOp


public class mainTele extends LinearOpMode {
    private ShooterController shooter;
    private SimpleLedController leds;
    private boolean isRed = true;
    
    // Declare our motors ----------------------------------------
    
    private DcMotor bl0, br1, fl2, fr3, cm4;
    private DcMotorEx launcher;
    private DcMotor intake, intake2;
    
    //Select Speed -------------------------------------------------
    
    double selectSpeed = 1.0;
    double intakePower = 1.0;
    
    boolean rightAPressedLast = false;
    boolean rightBPressedLast = false;
    boolean rightXPressedLast = false;
    boolean YPressedLast = false;
    
    // Test / control type
    //boolean 2player = true;
    
    
    //@Override
    public void runOpMode() throws InterruptedException {

        // Initialize motors from hardware map ------------------------
        
        bl0 = hardwareMap.get(DcMotor.class, "bl0");
        br1 = hardwareMap.get(DcMotor.class, "br1");
        fl2 = hardwareMap.get(DcMotor.class, "fl2");
        fr3 = hardwareMap.get(DcMotor.class, "fr3");
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake2 = hardwareMap.get(DcMotor.class, "intake2");
        //cm4 = hardwareMap.get(DcMotor.class, "cm4");
        
        // Encoder logic -------------------------------------------------
        launcher.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        
        // Set motor directions (adjust if movement is inverted) ----------
        fl2.setDirection(DcMotor.Direction.REVERSE);
        bl0.setDirection(DcMotor.Direction.REVERSE);
        fr3.setDirection(DcMotor.Direction.FORWARD);
        br1.setDirection(DcMotor.Direction.FORWARD);
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        
        // Set motor behavior ----------------------------------------------
        bl0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        //cm4.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("Ready to drive!");
        telemetry.update();
        
        shooter = new ShooterController(launcher, intake, intake2, this);
        
        // Initialize I2C LED controller - configure "ledController" as a REV Color/Range Sensor in your configuration
        /**
        try {
            leds = new SimpleLedController(hardwareMap, "ledController");

            // Set initial alliance color (red by default, can be changed based on match setup)
            leds.setAlliance(isRed);

            telemetry.addData("LED Status", leds.getStatus());
            telemetry.addData("LED Config", "Configure as REV Color/Range Sensor on I2C Bus 0");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("LED Error", "Failed to initialize LED controller");
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
            // Continue without LEDs if initialization fails
            leds = null;
        }
        */
        
        waitForStart();

        while (opModeIsActive()) {
            // Read joystick values --------------------------------------
            double y = -gamepad1.left_stick_y; // Forward is negative on the stick
            double x = gamepad1.left_stick_x;  // Strafe
            double rx = gamepad1.right_stick_x; // Rotation
    
            
            // Optional: Slow mode -----------------------------------------
            double speedMultiplier = gamepad2.right_bumper ? 0.4 : 1.0;

            // Calculate motor powers (mecanum math) -----------------------
            double flPower = (y + x + rx) * speedMultiplier;
            double blPower = (y - x + rx) * speedMultiplier;
            double frPower = (y - x - rx) * speedMultiplier;
            double brPower = (y + x - rx) * speedMultiplier;
            
            
            // Normalize powers if any exceed 1.0 ---------------------------
            double max = Math.max(1.0, Math.max(Math.abs(flPower),
                          Math.max(Math.abs(blPower), Math.max(Math.abs(frPower), Math.abs(brPower)))));
            flPower /= max;
            blPower /= max;
            frPower /= max;
            brPower /= max;
            
            //Change Speed ---------------------------------------------------
            if (gamepad1.b && !rightBPressedLast) {
                selectSpeed = (selectSpeed == 1.0) ? 0.5 : 1.0;
            }

            /**
            // Add LED control - press 'a' to toggle alliance color
            if (gamepad1.a && !rightAPressedLast) {
                isRed = !isRed;
                if (leds != null) {
                    leds.setAlliance(isRed);
                }
                telemetry.addData("Alliance", isRed ? "RED" : "BLUE");
            }
            */
            
            if (gamepad2.y && !YPressedLast) {
                    shooter.startShot(1, "short");
            }
            
            if (gamepad2.a && !rightAPressedLast) {
                    shooter.startShot(1, "long");
            }
            
            
            //Launcher Stuff :) ----------------------------------------------
            
            /*  USE THIS IF LAUNCHER DOESNT WORK, THIS WILL SET THE BOTTOM INTAKES TO GO BACKWARDS AND SHOOT THE BALL BACK OUT 
            IF WE ARE USING THIS WE WOULD TAKE THE BALLS AND STORE THEM IN OUR INTAKE AND THEN BRING THEM TO A CORNER WHERE WE WOULD SAFE GAURD
            THEM FOR OUR TEAM!
            
            if (gamepad1.x && !rightXPressedLast) {
                 intake2.setPower(-1);
            } else {
                 intake2.setPower(1);
            }
            
            */
            
            if (!shooter.isBusy()) {
                if (gamepad2.left_bumper)  {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }
                
                
                if (gamepad2.right_trigger > 0) {
                    telemetry.addLine("Velocity= " + launcher.getVelocity());
                    launcher.setPower(1.0);
                } else {
                    launcher.setPower(0);
                }
            }
        
            if (gamepad2.left_trigger > 0) {
                intake2.setPower(1);
            } else {
                intake2.setPower(0);
            }
            //Motor telemtry set up ------------------------------------------
            //int currentPosition = motor.getCurrentPosition();
            double currentTime = getRuntime();
            
            // small delay
            //int newPosition = motor.getCurrentPosition();
            
            
            // Speed selcetion -----------------------------------------------
            fl2.setPower(flPower*selectSpeed);
            bl0.setPower(blPower*selectSpeed);
            fr3.setPower(frPower*selectSpeed);
            br1.setPower(brPower*selectSpeed);
            
            // Display info on driver station --------------------------------
            telemetry.addLine("----- Motors -----");
            telemetry.addData("FL Power", flPower);
            telemetry.addData("FR Power", (-1 *frPower));
            telemetry.addData("BL Power", blPower);
            telemetry.addData("BR Power", brPower);
            
            telemetry.addLine("----- Motors Data -----");
            //telemetry.addData("Target");
            
            telemetry.addLine("----- Extras -----");
            telemetry.addData("X is pressed...", rightXPressedLast);
            telemetry.addLine("Intake Running");
            //telemetry.addData("LED Alliance", isRed ? "RED" : "BLUE");
            //telemetry.addData("LED Connected", leds.isConnected());
            telemetry.update();
            
            shooter.update();
        }
    }
        public void AlwaysWatching() {
        //THIS CODE WILL BE TO MAKE THE ROBOT FIND A APRIL TAG AND FACE TOWARDS IT! I MADE IT A FUNCTION SO WE CAN TURN IT ON AND OFF BUT AS YOU CAN
        //SEE THERE IS NO CODE IN HERE (IM SCARED OF APRIL TAGS)

            
        }
       
        public void FireBall() {
            
            launcher.setPower(-1);
            sleep(1500);
            intake.setPower(1);
            sleep(1000);
            launcher.setPower(0);
            intake.setPower(0);
        }
} 
