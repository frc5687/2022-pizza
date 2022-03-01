/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;

import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.config.Constants;
import org.frc5687.rapidreact.config.RobotMap;
import org.frc5687.rapidreact.util.OutliersContainer;

/**
 * DriveTrain moves the robot using four DiffSwerveModule instances.
 */
public class DriveTrain extends OutliersSubsystem {

    // Order we define swerve modules in kinematics
    // NB: must be same order as we pass to SwerveDriveKinematics
    public static final int NORTH_WEST = 0;
    public static final int SOUTH_WEST = 1;
    public static final int SOUTH_EAST = 2;
    public static final int NORTH_EAST = 3;

    // location of swerve modules on chassis
    private Translation2d NW_MODULE;
    private Translation2d SW_MODULE;
    private Translation2d SE_MODULE;
    private Translation2d NE_MODULE;

    private DiffSwerveModule _northWest;
    private DiffSwerveModule _southWest;
    private DiffSwerveModule _southEast;
    private DiffSwerveModule _northEast;

    private SwerveDriveKinematics _kinematics;
    private SwerveDriveOdometry _odometry;

    private double _PIDAngle;

    private AHRS _imu;
    private OI _oi;

    private HolonomicDriveController _controller;
    private ProfiledPIDController _angleController;

    /**
     * Create a DriveTrain
     * 
     * @param container
     * @param oi
     * @param imu
     */
    public DriveTrain(OutliersContainer container, OI oi, AHRS imu) {
        super(container);

        locateModules(); // figure out where swerve modules are on chassis

        try {
            _oi = oi;
            _imu = imu;

            _northWest =
                    new DiffSwerveModule(
                            NW_MODULE,
                            RobotMap.CAN.TALONFX.NORTH_WEST_OUTER,
                            RobotMap.CAN.TALONFX.NORTH_WEST_INNER,
                            RobotMap.DIO.NORTH_WEST,
                            Constants.DriveTrain.NORTH_WEST_OFFSET,
                            Constants.DriveTrain.NORTH_WEST_ENCODER_INVERTED);
            _southWest =
                    new DiffSwerveModule(
                            SW_MODULE,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_OUTER,
                            RobotMap.CAN.TALONFX.SOUTH_WEST_INNER,
                            RobotMap.DIO.SOUTH_WEST,
                            Constants.DriveTrain.SOUTH_WEST_OFFSET,
                            Constants.DriveTrain.SOUTH_WEST_ENCODER_INVERTED);
            _southEast =
                    new DiffSwerveModule(
                            SE_MODULE,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_INNER,
                            RobotMap.CAN.TALONFX.SOUTH_EAST_OUTER,
                            RobotMap.DIO.SOUTH_EAST,
                            Constants.DriveTrain.SOUTH_EAST_OFFSET,
                            Constants.DriveTrain.SOUTH_EAST_ENCODER_INVERTED);
            _northEast =
                    new DiffSwerveModule(
                            NE_MODULE,
                            RobotMap.CAN.TALONFX.NORTH_EAST_INNER,
                            RobotMap.CAN.TALONFX.NORHT_EAST_OUTER,
                            RobotMap.DIO.NORTH_EAST,
                            Constants.DriveTrain.NORTH_EAST_OFFSET,
                            Constants.DriveTrain.NORTH_EAST_ENCODER_INVERTED);
            // NB: it matters which order these are defined
            _kinematics =
                    new SwerveDriveKinematics(
                            _northWest.getModulePosition(),
                            _southWest.getModulePosition(),
                            _southEast.getModulePosition(),
                            _northEast.getModulePosition()
                        );
            _odometry = new SwerveDriveOdometry(_kinematics, getHeading());
            // TODO: rewrite HolonomicDriveController to use Waypoints (Pose, Vector)
            _controller =
                    new HolonomicDriveController(
                            new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
                            new PIDController(Constants.DriveTrain.kP, Constants.DriveTrain.kI, Constants.DriveTrain.kD),
                            new ProfiledPIDController(
                                    Constants.DriveTrain.kP,
                                    Constants.DriveTrain.kI,
                                    Constants.DriveTrain.kD,
                                    new TrapezoidProfile.Constraints(
                                            Constants.DriveTrain.PROFILE_CONSTRAINT_VEL, Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL)));
            _controller.setTolerance(Constants.DriveTrain.TOLERANCE);
            _angleController =
                    new ProfiledPIDController(
                            Constants.DriveTrain.ANGLE_kP,
                            Constants.DriveTrain.ANGLE_kI,
                            Constants.DriveTrain.ANGLE_kD,
                            new TrapezoidProfile.Constraints(
                                    Constants.DriveTrain.PROFILE_CONSTRAINT_VEL, Constants.DriveTrain.PROFILE_CONSTRAINT_ACCEL));
            _angleController.enableContinuousInput(-Math.PI / 2.0, Math.PI / 2.0);
        } catch (Exception e) {
            error(e.getMessage());
        }
    }

    public void setModuleState(DiffSwerveModule module, SwerveModuleState state) {
        module.setIdealState(state);
    }

    public void setAllModuleStates(SwerveModuleState[] states) {
        setModuleState(_northWest, states[NORTH_WEST]);
        setModuleState(_southWest, states[SOUTH_WEST]);
        setModuleState(_southEast, states[SOUTH_EAST]);
        setModuleState(_northEast, states[NORTH_EAST]);
    }

    /**
     * Method to set correct module speeds and angle based on wanted vx, vy, vomega
     *
     * @param vx velocity in x direction
     * @param vy velocity in y direction
     * @param vomega angular velocity (rotating speed)
     * @param fieldRelative forward is always forward no matter orientation of robot.
     */
    public void drive(double vx, double vy, double vomega, boolean fieldRelative) {
        if (
            (Math.abs(vx) < Constants.DriveTrain.DEADBAND_TRANSLATION) &&
            (Math.abs(vy) < Constants.DriveTrain.DEADBAND_TRANSLATION) &&
            (Math.abs(vomega) < Constants.DriveTrain.DEADBAND_ROTATION)
        ) {
            // stop moving
            setModuleState(_northWest,
                    new SwerveModuleState(0, new Rotation2d(_northWest.getModuleAngle())));
            setModuleState(_southWest,
                    new SwerveModuleState(0, new Rotation2d(_southWest.getModuleAngle())));
            setModuleState(_southEast,
                    new SwerveModuleState(0, new Rotation2d(_southEast.getModuleAngle())));
            setModuleState(_northEast,
                    new SwerveModuleState(0, new Rotation2d(_northEast.getModuleAngle())));
            _PIDAngle = getHeading().getRadians();
            _angleController.reset(_PIDAngle);
        } else if (Math.abs(vomega) > 0) {
            SwerveModuleState[] swerveModuleStates =
                    _kinematics.toSwerveModuleStates(
                            fieldRelative
                                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                            vx, vy, vomega, getHeading())
                                    : new ChassisSpeeds(vx, vy, vomega));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS);
            setAllModuleStates(swerveModuleStates);
            _PIDAngle = getHeading().getRadians();
            _angleController.reset(_PIDAngle);
        } else {
            SwerveModuleState[] swerveModuleStates =
                    _kinematics.toSwerveModuleStates(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    vx,
                                    vy,
                                    _angleController.calculate(
                                            getHeading().getRadians(), _PIDAngle),
                                    new Rotation2d(_PIDAngle)));
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS);
            setAllModuleStates(swerveModuleStates);
        }
    }

    // use for modules as controller is running at 200Hz.
    public void controllerPeriodic() {
        _northWest.periodic();
        _southWest.periodic();
        _southEast.periodic();
        _northEast.periodic();
    }

    @Override
    public void periodic() {
        _odometry.update(
                getHeading(),
                _northWest.getState(),
                _southWest.getState(),
                _southEast.getState(),
                _northEast.getState()
            );
    }

    public void snap(Rotation2d theta){
        poseFollower(_odometry.getPoseMeters(), theta, Constants.SnapPose.SNAP_LRF);
    }

    /**
     * Control each swerve module to reach the desired pose and velocity
     * 
     * //TODO: fix adjustedSpeeds to use Waypoints (Pose, Vector)
     * 
     * @param pose xPos in meters, yPos in meters, omega in radians
     * @param heading omega in radians
     * @param vel in m/s
     */
    public void poseFollower(Pose2d pose, Rotation2d heading, double vel) {
        ChassisSpeeds adjustedSpeeds = _controller.calculate(_odometry.getPoseMeters(), pose, vel, heading);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveTrain.MAX_MPS);
        setAllModuleStates(moduleStates);
    }

    /**
     * Check if robot is at theta rotation
     * 
     * Rotation is CCW +.
     * 
     * TODO: // verify that yaw is CW +
     * 
     * @param theta
     * @return true if rotation equals theta
     */
    public boolean isAtRotation(Rotation2d theta){
        // Rotation2d rotation = new Rotation2d(Math.toRadians(_imu.getYaw()));
        Rotation2d rotation = Rotation2d.fromDegrees(-getYaw());
        return (rotation == theta);
    }

    /** Check if robot is at pose
     * 
     * <p> NOTE: set TOLERANCE in Constants
     * 
     * @param poseRef reference pose
     * @return true if current pose within tolerance of reference pose
     */
    public boolean isAtPose(Pose2d poseRef) {
        // Pose error
        Pose2d posError = poseRef.relativeTo(getOdometryPose());
        Rotation2d rotError = poseRef.getRotation().minus(Rotation2d.fromDegrees(-getYaw()));
        // Pose tolerance
        double xTol = Constants.DriveTrain.TOLERANCE.getX();
        double yTol = Constants.DriveTrain.TOLERANCE.getY();
        double omegaTol = Constants.DriveTrain.TOLERANCE.getRotation().getRadians();
        // Is error below tolerance?
        return
            (Math.abs(posError.getX()) < xTol) &&
            (Math.abs(posError.getY()) < yTol) &&
            (Math.abs(rotError.getRadians()) < omegaTol);
    }

    public double getYaw() {
        return _imu.getYaw();
    }

    /**
     * Get heading of robot according to IMU
     * 
     * Note: yaw is CW, but heading is CCW
     * 
     * TODO: confirm that CW and CCW is documented correctly for getHeading()
     * 
     * @return heading as a Rotation2d
     */
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-getYaw());
    }

    public void resetYaw() {
        _imu.reset();
    }

    public SwerveDriveKinematicsConstraint getKinematicConstraint() {
        return new SwerveDriveKinematicsConstraint(_kinematics, Constants.DriveTrain.MAX_MPS);
    }

    public TrajectoryConfig getConfig() {
        return new TrajectoryConfig(Constants.DriveTrain.MAX_MPS, Constants.DriveTrain.MAX_MPSS)
                .setKinematics(_kinematics)
                .addConstraint(getKinematicConstraint());
    }

    public void trajectoryFollower(Trajectory.State goal, Rotation2d heading) {
        ChassisSpeeds adjustedSpeeds =
                _controller.calculate(_odometry.getPoseMeters(), goal, heading);
        SwerveModuleState[] moduleStates = _kinematics.toSwerveModuleStates(adjustedSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DifferentialSwerveModule.MAX_MODULE_SPEED_MPS);
        setAllModuleStates(moduleStates);
    }

    public Pose2d getOdometryPose() {
        return _odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d position) {
        _odometry.resetPosition(position, getHeading());
    }

    public void startModules() {
        _northWest.start();
        _southWest.start();
        _southEast.start();
        _northEast.start();
    }

    // Helper methods

    /** Calculate position vectors for the swerve module kinematics */
    private void locateModules() {

        // Location of each swerve module from center of robot
        // NB: signs depend on orientation of robot when IMU inits

        /** Figure out signs of compass headings
         * 
         * If we put robot on field with robot North facing field North,
         * Bob's your uncle.
         * 
         * But put robot on field with robot North facing field East,
         * this is what happens.
         * 
         * (X, Y):
         *   X is N or S, N is +
         *   Y is W or E, W is +
         * 
         *   NW (+,+)  NE (+,-)
         * 
         *   SW (-,+)  SE (-,-)
         * 
         * NOTE: WPI kinematics classes assume robot NORTH = field EAST
         * 
         */

        int nw_x;
        int sw_x;
        int se_x;
        int ne_x;
        int nw_y;
        int sw_y;
        int se_y;
        int ne_y;

        switch(Constants.DriveTrain.ROBOT_FACING) {
            case NORTH:
                nw_x = -1;
                sw_x = -1;
                se_x = 1;
                ne_x = 1;
                nw_y = 1;
                sw_y = -1;
                se_y = -1;
                ne_y = 1;
                break;
            case EAST:
                nw_x = 1;
                sw_x = -1;
                se_x = -1;
                ne_x = 1;
                nw_y = 1;
                sw_y = 1;
                se_y = -1;
                ne_y = -1;
                break;
            case SOUTH:
                nw_x = 1;
                sw_x = 1;
                se_x = -1;
                ne_x = -1;
                nw_y = -1;
                sw_y = 1;
                se_y = 1;
                ne_y = -1;
                break;
            case WEST:
                nw_x = -1;
                sw_x = 1;
                se_x = 1;
                ne_x = -1;
                nw_y = -1;
                sw_y = -1;
                se_y = 1;
                ne_y = 1;
                break;
            default: // robot North = field North
                nw_x = -1;
                sw_x = -1;
                se_x = 1;
                ne_x = 1;
                nw_y = 1;
                sw_y = -1;
                se_y = -1;
                ne_y = 1;
        }

        NW_MODULE =
            new Translation2d(
                nw_x * Constants.DriveTrain.SWERVE_NS_POS,
                nw_y * Constants.DriveTrain.SWERVE_WE_POS );
        SW_MODULE =
            new Translation2d(
                sw_x * Constants.DriveTrain.SWERVE_NS_POS,
                sw_y * Constants.DriveTrain.SWERVE_WE_POS );
        SE_MODULE =
            new Translation2d(
                se_x * Constants.DriveTrain.SWERVE_NS_POS,
                se_y * Constants.DriveTrain.SWERVE_WE_POS );
        NE_MODULE =
            new Translation2d(
                ne_x * Constants.DriveTrain.SWERVE_NS_POS,
                ne_y * Constants.DriveTrain.SWERVE_WE_POS );
    }

    @Override
    public void updateDashboard() {
        metric("vx", _oi.getDriveX());
        metric("vy", _oi.getDriveY());
        metric("NW/Encoder Angle", _northWest.getModuleAngle());
        metric("SW/Encoder Angle", _southWest.getModuleAngle());
        metric("SE/Encoder Angle", _southEast.getModuleAngle());
        metric("NE/Encoder Angle", _northEast.getModuleAngle());

        metric("SW/Predicted Angle", _southWest.getPredictedAzimuthAngle());

        metric("SW/Encoder Azimuth Vel", _southWest.getAzimuthAngularVelocity());
        metric("SW/Predicted Azimuth Vel", _southWest.getPredictedAzimuthAngularVelocity());

        metric("SW/Encoder Wheel Vel", _southWest.getWheelVelocity());
        metric("SW/Predicted Wheel Vel", _southWest.getPredictedWheelVelocity());
        Pose2d odometry=getOdometryPose();
        metric("Odometry/x", odometry.getX());
        metric("Odometry/y", odometry.getY());
        metric("Odometry/angle", odometry.getRotation().getDegrees());
        metric("Odometry/theta", odometry.getRotation().getRadians() / Math.PI);
        metric("Odometry/Pose", getOdometryPose().toString());
    }

}
