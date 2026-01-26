package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Pedro Pathing Constants for Kraken Koders robot.
 *
 * This is the starting configuration - you will need to run the tuning
 * OpModes to calibrate these values for your specific robot.
 */
public class Constants {

    // Follower constants - these get configured during tuning
    public static FollowerConstants followerConstants = new FollowerConstants();

    // Path constraints: (max velocity multiplier, max velocity, max accel, max angular velocity)
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /**
     * Creates and configures a Follower.
     * Call this in your OpMode's init() method.
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .build();
    }
}
