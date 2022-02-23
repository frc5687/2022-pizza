/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.rapidreact.util.AutoChooser;
import org.frc5687.rapidreact.util.OutliersContainer;

import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Intake;

import org.frc5687.rapidreact.commands.DriveOI;
import org.frc5687.rapidreact.commands.DriveAuto;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.commands.auto.ZeroBallAuto;
import org.frc5687.rapidreact.commands.auto.DeployIntake;
import org.frc5687.rapidreact.commands.auto.AutoWait;

/**
 * TODO: explain RobotContainer class
 */
public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AutoChooser _autoChooser;
    private AHRS _imu;

    private Robot _robot;
    private DriveTrain _driveTrain;
    private Indexer _indexer;
    private Intake _intake;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    // Initialization methods

    public void init() {
        _oi = new OI();
        _autoChooser = new AutoChooser();
        
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200); //Config the NavX
        _driveTrain = new DriveTrain(this, _oi, _imu);
        _indexer = new Indexer(this);
        _intake = new Intake(this);

        info("Running RobotContainer.init()");

        // DriveTrain's default command is DriveOI
        setDefaultCommand(_driveTrain, new DriveOI(_driveTrain, _oi));
        // Run the control loop for each individual swerve drive unit every 5 ms.
        // DriveTrain has four DiffSwerveModules.
        // controllerPeriodic calls the periodic for each of them.
        _robot.addPeriodic(this::controllerPeriodic, Constants.DifferentialSwerveModule.kDt, 0.005);
        //_driveTrain.resetOdometry(zeroBall.getInitialPose());
        //_oi.initializeButtons(_driveTrain, zeroBall);
    }

    @Override
    public void disabledInit() {
        //Runs once during disabled
    }

    @Override
    public void autonomousInit() {
        //Runs once during auto
        info("Running RobotContainer.autonomousInit()");

        // Set state of subsystems
        _indexer.setState(Indexer.IndexerState.DEPLOYED);
        _intake.setState(Intake.IntakeState.STOWED);
    }

    @Override
    public void teleopInit() {
        //Runs at the start of teleop
    }

    // Periodic methods

    public void disabledPeriodic() {
        //Runs every 20ms during disabled
    }

    public void periodic() {
        //Runs every 20ms
    }

    // Helper methods

    /** 
     * Helper function to wrap DriveTrain.controllerPeriodic.  
     * This allows us to pass nulls during initial development
     * without breaking.
    */
    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.controllerPeriodic();
        }
    }

    /**
     * Helper function to wrap CommandScheduler.setDefaultCommand.  This allows us to pass nulls during initial development
     * without breaking.
     * 
     * @param subSystem
     * @param command
     */
    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    /** Return a SequentialCommandGroup to run during auto */
    public Command getAutonomousCommand() {

        AutoWait _waitOneSecond;
        DeployIntake _deployIntake;
        DriveAuto _driveToA;
        DriveAuto _driveToB;

        Pose2d _waypoint;
        Rotation2d _heading;
        Double _velocity;
        
        _waitOneSecond = new AutoWait(1.0);
        _deployIntake = new DeployIntake(_intake);

        // destination A
        _waypoint = new Pose2d(-1.0, -1.0, new Rotation2d(0.0));
        _heading = new Rotation2d(0.0);
        _velocity = 0.1;
        _driveToA = getAutoDriveCommand(_driveTrain, _waypoint, _heading, _velocity);

        // destination B
        _waypoint = new Pose2d(0.0, -1.0, new Rotation2d(Math.PI));
        _heading = new Rotation2d(Math.PI);
        _velocity = 0.2;
        _driveToB = getAutoDriveCommand(_driveTrain, _waypoint, _heading, _velocity);

        return new SequentialCommandGroup(
            _waitOneSecond,
            _deployIntake,
            _driveToA,
            _waitOneSecond,
            _driveToB
        );

    }

    /** Return a drive to destination command */
    public DriveAuto getAutoDriveCommand(
        DriveTrain driveTrain,
        Pose2d wayPoint,
        Rotation2d heading,
        Double velocity
        ) {
        return new DriveAuto(driveTrain, wayPoint, heading, velocity);
    }

    @Override
    public void updateDashboard() {
        //Updates the driver station
        _driveTrain.updateDashboard();
        metric("AutoChooser", _autoChooser.getSelectedMode().getValue());
        _autoChooser.updateDashboard();
    }
}
