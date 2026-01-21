package org.firstinspires.ftc.teamcode.constants.DriveConstants;

public class DriveConstants {
    public static final double TICKS_PER_REV = 383.6; // GoBILDA 312 RPM motor, example TODO: UPDATE FOR OUR MOTORS
    public static final double WHEEL_DIAMETER_INCHES = 4.0; // TODO: UPDATE FOR OUR WHEELS
    public static final double GEAR_RATIO = 1.0; // output / motor

    public static final double TICKS_PER_INCH =
            (TICKS_PER_REV * GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_INCHES);
}
