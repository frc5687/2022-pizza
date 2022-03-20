/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact.config;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Drive train tuning values (such as speeds, PID gains, etc.).
 * 
 * <ul>
 *  <li> what angle the wheels point,
 *  <li> which direction the wheels turn,
 *  <li> how sensitive the joystick is,
 *  <li> maximum speed and acceleration
 * </ul>
 * 
 * @see Constants
 */
public class DriveTuning {

    // Which field direction is robot North facing when IMU inits
    // Field East is away from our drive team.
    public static enum Direction {
        NORTH,
        EAST,
        SOUTH,
        WEST
    }
    public static final Direction ROBOT_FACING = Direction.EAST;

    // Control

    // Avoid unintentional joystick movememnt
    public static final double DEADBAND_TRANSLATION = 0.2;
    public static final double DEADBAND_ROTATION = 0.2;

    // Maximum rates of motion
    public static final double MAX_MPS = 3.5; // Max speed of robot (m/s)
    public static final double MAX_ANG_VEL = Math.PI * 1.5; // Max rotation rate of robot (rads/s)
    public static final double MAX_MPSS = 0.5; // Max acceleration of robot (m/s^2)
    public static final double SLEW_LIMIT_X = 3.0; // rate of change limit, units per s
    public static final double SLEW_LIMIT_Y = 3.0; // rate of change limit, units per s

    // PID controller settings
    public static final Pose2d TOLERANCE = new Pose2d(0.02, 0.02, new Rotation2d(0.05));
    public static final double ANGLE_kP = 3.5;
    public static final double ANGLE_kI = 0.0;
    public static final double ANGLE_kD = 0.0;
    public static final double kP = 9.5;
    public static final double kI = 0.0;
    public static final double kD = 0.5;
    public static final double PROFILE_CONSTRAINT_VEL = 3.0 * Math.PI;
    public static final double PROFILE_CONSTRAINT_ACCEL = Math.PI;

    // Correct for hardware error in swerve module headings
    // Should be 0 if hardware is working correctly
    public static final double NORTH_WEST_OFFSET = 0; // radians
    public static final double SOUTH_WEST_OFFSET = 0; // radians
    public static final double SOUTH_EAST_OFFSET = 0; // radians
    public static final double NORTH_EAST_OFFSET = 0; // radians

    // In case encoder is measuring rotation in the opposite direction we expect.
    public static final boolean NORTH_WEST_ENCODER_INVERTED = false;
    public static final boolean SOUTH_WEST_ENCODER_INVERTED = false;
    public static final boolean SOUTH_EAST_ENCODER_INVERTED = false;
    public static final boolean NORTH_EAST_ENCODER_INVERTED = false;

}