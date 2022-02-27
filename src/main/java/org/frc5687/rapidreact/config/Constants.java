/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact.config;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import org.frc5687.rapidreact.util.*;

/** Define globally-accessible robot constants (such as speeds, PID gains, etc.).
 * 
 * @see RobotMap RobotMap for Robot port mappings (CAN IDs, etc.)
 * 
 * @see JoystickMap ButtonMap for Joystick and Gamepad settings and button mappings
 * 
 * @see DriveTuning DriveTuning for drive train tuning values
 * 
 * @see AutoConfig AutoConfig for auto mode configurations
 */
public class Constants {

    /** Conventions for constants
     * 
     * - Field position and robot size measured in meters.
     * - Time in seconds unless otherwise noted.
     * - Angles in radians unless otherwise noted.
     * - Localization position is field reference relative to robot starting position.
     * - Field reference North is away from drive team, East is to drive team's right.
     * - Robot reference North is direction catapult shoots, South is direction intake deploys.
     * 
     * Q: Are angles given clockwise (+ turns right) or counter-clockwise (+ turns left)?
     */

    // NOTE: see DriveTuning for configuring direction robot drives.
    // You must decide whether robot North = field North or another field direction.
    // We have been placing robot on field with robot North = field East.
    // Doing this makes our code a bit confusing.
    // Field East is away from our drive team.

    // Declare constants as public static final so that they are globally accessible
    // and cannot be changed.

    public static final String ROBOT_NAME = "pizza bot";
    public static final OutliersContainer.IdentityMode IDENTITY = 
        OutliersContainer.IdentityMode.programming;
    public static final RioLogger.LogLevel DS_LOG_LVL = RioLogger.LogLevel.info;
    public static final RioLogger.LogLevel USB_LOG_LVL = RioLogger.LogLevel.warn;

    public static final int TICKS_PER_UPDATE = 1; // increase to log less frequently
    public static final double METRIC_FLUSH_PERIOD = 1.0; // seconds
    public static final double UPDATE_PERIOD = 0.02; // seconds?
    public static final double EPSILON = 0.00001; // for calculations to make math work

    // Separate constants into individual inner classes corresponding
    // to subsystems or robot modes, to keep variable names shorter.

    /** Constants for driving robot
     * 
     * <p>Set physical characteristics of drive train.
     * 
     * @see DriveTuning DriveTuning for drive train tuning values
     */
    public static class DriveTrain {

        /** Use compass headings to reference swerve modules.
         * 
         * When looking down at top of robot:
         * 
         *          N
         *          |
         *      W -- -- E
         *          |
         *          S
         * 
         * When robot is flipped over on its back:
         * 
         *          N
         *          |
         *      E -- -- W
         *          |
         *          S
         */

        // Size of the robot chassis in meters
        public static final double WIDTH = 0.6223; // meters
        public static final double LENGTH = 0.6223; // meters

        /**
         * Swerve modules are on four corners of robot:
         * 
         * NW  <- Width of robot ->  NE
         *             / \
         *              |
         *        Length of robot
         *              |
         *             \ /
         *  SW                       SE
         */

        // Distance of swerve modules from center of robot
        public static final double SWERVE_NS_POS = LENGTH / 2.0;
        public static final double SWERVE_WE_POS = WIDTH / 2.0;

        // Values set in DriveTuning

        // Which way robot is facing when IMU inits
        public static final DriveTuning.Direction ROBOT_FACING = DriveTuning.ROBOT_FACING;

        // Control

        // Avoid unintentional joystick movememnt
        public static final double DEADBAND_TRANSLATION = DriveTuning.DEADBAND_TRANSLATION;
        public static final double DEADBAND_ROTATION = DriveTuning.DEADBAND_ROTATION;

        // Maximum rates of motion
        public static final double MAX_MPS = DriveTuning.MAX_MPS;
        public static final double MAX_ANG_VEL = DriveTuning.MAX_ANG_VEL;
        public static final double MAX_MPSS = DriveTuning.MAX_MPSS;

        // PID controller settings
        public static final double ANGLE_kP = DriveTuning.ANGLE_kP;
        public static final double ANGLE_kI = DriveTuning.ANGLE_kI;
        public static final double ANGLE_kD = DriveTuning.ANGLE_kD;

        public static final double kP = DriveTuning.kP;
        public static final double kI = DriveTuning.kI;
        public static final double kD = DriveTuning.kD;
        public static final double PROFILE_CONSTRAINT_VEL = DriveTuning.PROFILE_CONSTRAINT_VEL;
        public static final double PROFILE_CONSTRAINT_ACCEL = DriveTuning.PROFILE_CONSTRAINT_ACCEL;

        // Should be 0, but can correct for hardware error in swerve module headings here.
        public static final double NORTH_WEST_OFFSET = DriveTuning.NORTH_WEST_OFFSET;
        public static final double SOUTH_WEST_OFFSET = DriveTuning.SOUTH_WEST_OFFSET;
        public static final double SOUTH_EAST_OFFSET = DriveTuning.SOUTH_EAST_OFFSET;
        public static final double NORTH_EAST_OFFSET = DriveTuning.NORTH_EAST_OFFSET;

        // In case encoder is measuring rotation in the opposite direction we expect.
        public static final boolean NORTH_WEST_ENCODER_INVERTED = DriveTuning.NORTH_WEST_ENCODER_INVERTED;
        public static final boolean SOUTH_WEST_ENCODER_INVERTED = DriveTuning.SOUTH_WEST_ENCODER_INVERTED;
        public static final boolean SOUTH_EAST_ENCODER_INVERTED = DriveTuning.SOUTH_EAST_ENCODER_INVERTED;
        public static final boolean NORTH_EAST_ENCODER_INVERTED = DriveTuning.NORTH_EAST_ENCODER_INVERTED;

    }

    /** Constants for swerve module hardware.
     * 
     * <p>These constants relate to the physical characteristics of the swerve module design.
     */
    public static class DifferentialSwerveModule {

        // update rate of our modules 5ms.
        // used in RobotContainer to schedule periodic update
        public static final double kDt = 0.005; // seconds

        public static final double FALCON_FREE_SPEED =
                Units.rotationsPerMinuteToRadiansPerSecond(6380);
        public static final int TIMEOUT = 200; // units?
        public static final double GEAR_RATIO_WHEEL = 6.46875;
        public static final double GEAR_RATIO_STEER = 9.2;
        public static final double FALCON_RATE = 600.0;
        public static final double WHEEL_RADIUS = 0.0508; // Meters with compression.
        public static final double MAX_MODULE_SPEED_MPS =
                (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL) * WHEEL_RADIUS;
        public static final double TICKS_TO_ROTATIONS = 2048.0;
        public static final double VOLTAGE = 12.0;
        public static final double FEED_FORWARD = VOLTAGE / (FALCON_FREE_SPEED / GEAR_RATIO_WHEEL);

        public static final boolean ENABLE_CURRENT_LIMIT = true;
        public static final double CURRENT_LIMIT = 30.0;
        public static final double CURRENT_THRESHOLD = 30.0;
        public static final double CURRENT_TRIGGER_TIME = 0.0;

        // Create Parameters for DiffSwerve State Space
        public static final double INERTIA_WHEEL = 0.005;
        public static final double INERTIA_STEER = 0.004;
        // A weight for how aggressive each state should be ie. 0.08 radians will try to control the
        // angle more aggressively than the wheel angular velocity.
        public static final double Q_AZIMUTH_ANG_VELOCITY = 1.1; // radians per sec
        public static final double Q_AZIMUTH = 0.08; // radians
        public static final double Q_WHEEL_ANG_VELOCITY = 5; // radians per sec
        // This is for Kalman filter which isn't used for azimuth angle due to angle wrapping.
        // Model noise are assuming that our model isn't as accurate as our sensors.
        public static final double MODEL_AZIMUTH_ANGLE_NOISE = .1; // radians
        public static final double MODEL_AZIMUTH_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        public static final double MODEL_WHEEL_ANG_VELOCITY_NOISE = 5.0; // radians per sec
        // Noise from sensors. Falcon With Gearbox causes us to have more uncertainty so we increase
        // the noise.
        public static final double SENSOR_AZIMUTH_ANGLE_NOISE = 0.01; // radians
        public static final double SENSOR_AZIMUTH_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double SENSOR_WHEEL_ANG_VELOCITY_NOISE = 0.1; // radians per sec
        public static final double CONTROL_EFFORT = VOLTAGE;
    }

    public static class AutoPath {
        public static class ZBLeft1ballT {
                public static final List<Pose2d> waypoints =
                     Arrays.asList(
                             new Pose2d (9.9, 2.48, Rotation2d.fromDegrees(-43)),
                             new Pose2d (10.8, 1.65, Rotation2d.fromDegrees(-46)));
                        
        }
        public static class ZBRight1ballT {
                public static final List<Pose2d> waypoints =
                     Arrays.asList(
                             new Pose2d (10.5, 4.1, Rotation2d.fromDegrees(0)),
                             new Pose2d (12.2, 4.1, Rotation2d.fromDegrees(-46)));
                        
        }
        public static class ZBLeft2ballT {
                public static final List<Pose2d> waypoints =
                     Arrays.asList(
                             new Pose2d (9.6, 5.9, Rotation2d.fromDegrees(44.3)),
                             new Pose2d (10.6, 6.9, Rotation2d.fromDegrees(46.7)));
                        
        }
        public static class ZBRight2ballT {
                public static final List<Pose2d> waypoints =
                     Arrays.asList(
                             new Pose2d (8.3, 6, Rotation2d.fromDegrees(90)),
                             new Pose2d (8.3, 7.4, Rotation2d.fromDegrees(90)));
                        
        }
        /*public static class 2BLeft1ballT {
                public static final List<Pose2d> waypoints =
                     Arrays.asList(
                             new Pose2d (9.9, 2.6, Rotation2d.fromDegrees(-22.9)),
                             new Pose2d (11.6, -15.3, Rotation2d.fromDegrees(90)));
        }*/
     }

    public static class SnapPose {
        public static final double SNAP_LRF = 3.5;
    }
    public static class UDPJetson {
        public static final int BUFFER = 1024;
    }

    public static class Catapult {
        public static final long DELAY = 100; // ms

        public static final boolean SPRING_MOTOR_INVERTED = false;
        public static final boolean WINCH_MOTOR_INVERTED = false;

        public static final int COUNTS_PER_REVOLUTION = 8196;

        public static final double GEAR_REDUCTION = 64.0;
//        public static final double GEAR_REDUCTION_VP = 50.0;

        public static final double BABY_NEO_RAD_PER_SEC = Units.rotationsPerMinuteToRadiansPerSecond(11710);
        public static final double NEO_RAD_PER_SEC = Units.rotationsPerMinuteToRadiansPerSecond(5500);
        public static final double MAX_SPEED_WITH_GEAR_BOX = BABY_NEO_RAD_PER_SEC / GEAR_REDUCTION;
//        public static final double MAX_SPEED_WITH_GEAR_BOX_VP = BABY_NEO_RAD_PER_SEC / GEAR_REDUCTION_VP;
        public static final double SPRING_WINCH_DRUM_CIRCUMFERENCE = Units.inchesToMeters(0.875) * Math.PI; // meters
        public static final double ARM_WINCH_DRUM_CIRCUMFERENCE = Units.inchesToMeters(1.437) * Math.PI; // meters

        // Physical characteristics
        public static final double POUND_PER_IN_TO_NEWTON_PER_METER = 0.0057101471627692;
        public static final double SPRING_RATE = 11.2 * POUND_PER_IN_TO_NEWTON_PER_METER; // Newtons per meter.
        public static final double ARM_LENGTH = Units.inchesToMeters(20.9);
        public static final double LEVER_ARM_LENGTH = Units.inchesToMeters(6.0);
        public static final double ARM_MASS = 1.2; //kg
        public static final double BALL_INERTIA = 0.0432;
        public static final double ROD_INERTIA = (1.0 / 3.0) * ARM_MASS * (ARM_LENGTH * ARM_LENGTH);
        public static final double INERTIA_OF_ARM = ROD_INERTIA + BALL_INERTIA;

        public static final double STOWED_ANGLE = Units.degreesToRadians(90 + 24.724);

        //Linear regression constants
        public static final double LINEAR_REGRESSION_SLOPE = 3.6073; // meters and radians
        public static final double LINEAR_REGRESSION_OFFSET = -0.0217; //meters and radians


        // Spring Linear actuator limits
        public static final double SPRING_BOTTOM_LIMIT = 0; //TODO: Real values

        // Winch actuator limits
        public static final double WINCH_BOTTOM_LIMIT = 0;

        // Controller Parameters
        // spring
        public static final double SPRING_kP = 40.0; // Always start with kP
        public static final double SPRING_kI = 28.0; // If possible avoid kI
        public static final double SPRING_kD = 0.0; // 2nd Kd
        public static final double MAX_SPRING_VELOCITY_MPS = (MAX_SPEED_WITH_GEAR_BOX / (2 * Math.PI)) * SPRING_WINCH_DRUM_CIRCUMFERENCE; // divide by 2 PI as that is one rotation.
        public static final double MAX_SPRING_ACCELERATION_MPSS = MAX_SPRING_VELOCITY_MPS * 20; // heuristic.
        public static final double SPRING_IZONE = 30.0;
        public static final double SPRING_TOLERANCE = 0.001; // m
        public static final double SPRING_DISPLACEMENT_FACTOR = -0.0; // TODO: magic number
        // winch
        public static final double WINCH_kP = 20.0; // Always start with kP
        public static final double WINCH_kI = 0.0; // If possible avoid kI
        public static final double WINCH_kD = 0.0; // 2nd Kd
//        public static final double MAX_WINCH_VELOCITY_MPS = ((NEO_RAD_PER_SEC / GEAR_REDUCTION)/ (2 * Math.PI)) * ARM_WINCH_DRUM_CIRCUMFERENCE; // m/s
        public static final double MAX_WINCH_VELOCITY_MPS = (MAX_SPEED_WITH_GEAR_BOX / (2 * Math.PI)) * ARM_WINCH_DRUM_CIRCUMFERENCE; // m/s
        public static final double MAX_WINCH_ACCELERATION_MPSS = MAX_WINCH_VELOCITY_MPS * 20.0; // heuristic.
        public static final double WINCH_TOLERANCE = 0.001; // m

        // Shoot constants
        public static final double LOWERING_SPEED = 1.0;
        public static final double SPRING_ZERO_SPEED = -0.5;
        public static final double REMOVE_BALL_WINCH_GOAL = 0.1;
        public static final double REMOVE_BALL_SPRING_GOAL = 0.05;
        public static final double INITIAL_BALL_WINCH_GOAL = 0.245;
        public static final double INITIAL_BALL_SPRING_GOAL = 0.08;


    }
    public static class IntakeBlocker {
        public static final double DOWN_POSITION = 180;
        public static final double UP_POSITION = 50;
    }

    public static class Intake {
        public static final boolean INVERTED = false;
        public static final double ROLLER_IDLE_SPEED = 0.0;
        public static final double ROLLER_INTAKE_SPEED = 1.0;
    }

    /** Constants for autonomous mode
     * 
     * @see AutoConfig
     */
    public static class Auto {
        public static final double DRIVETRAIN_POWER = AutoConfig.DRIVETRAIN_POWER;
        public static final AutoConfig.Mode AUTO_MODE = AutoConfig.AUTO_MODE;
        public static final AutoConfig.Position AUTO_POSITION = AutoConfig.AUTO_POSITION;
    }
}