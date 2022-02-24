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
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.commands.auto.DeployIntake;
import org.frc5687.rapidreact.commands.auto.DriveToPose;
import org.frc5687.rapidreact.commands.auto.Wait;

/**
 * Define subsystems and commands, bind commands to triggering events (such as buttons),
 * and specify which command to run in autonomous mode.
 * 
 * <p>Why separate Robot and RobotContainer classes?
 * See https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html
 */
public class RobotContainer extends OutliersContainer {

    private Robot _robot;

    private OI _oi;
    private AutoChooser _autoChooser;
    private AHRS _imu;

    private DriveTrain _driveTrain;
    private Indexer _indexer;
    private Intake _intake;

    /**
     * Create RobotContainer 
     * 
     * @param robot
     * @param identityMode
     */
    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    // Initialization methods

    public void init() {
        info("Running RobotContainer.init()");

        _oi = new OI();
        _autoChooser = new AutoChooser();

        // 
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200); //Config the NavX
        _driveTrain = new DriveTrain(this, _oi, _imu);
        _indexer = new Indexer(this);
        _intake = new Intake(this);

        _oi.initializeButtons(_driveTrain, _indexer, _intake);

        // DriveTrain's default command is DriveOI
        setDefaultCommand(_driveTrain, new DriveOI(_driveTrain, _oi));
        // Run the control loop for each individual swerve drive unit every 5 ms.
        // DriveTrain has four DiffSwerveModules.
        // controllerPeriodic calls the periodic for each of them.
        _robot.addPeriodic(this::controllerPeriodic, Constants.DifferentialSwerveModule.kDt, 0.005);

        _imu.reset();
    }

    @Override
    public void disabledInit() {
        // Run once when entering disabled mode
    }

    @Override
    public void autonomousInit() {
        // Run once when entering auto mode
        info("Running RobotContainer.autonomousInit()");

        // Set state of subsystems
        _indexer.setState(Indexer.IndexerState.DEPLOYED);
        _intake.setState(Intake.IntakeState.STOWED);
    }

    @Override
    public void teleopInit() {
        // Run once when entering teleop mode
    }

    // Periodic methods

    public void disabledPeriodic() {
        // Run every 20ms during disabled
    }

    public void periodic() {
        // Run every 20ms
    }

    // Helper methods

    /** 
     * Helper function to wrap DriveTrain.controllerPeriodic.  
     * _driveTrain can be null during initial development
    */
    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.controllerPeriodic();
        }
    }

    /**
     * Helper function to wrap CommandScheduler.setDefaultCommand.
     * 
     * @param subSystem can be null
     * @param command can be null
     */
    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    /**
     * Return sequence of commands to run during auto
     * 
     * @return SequentialCommandGroup
    */
    public Command getAutonomousCommand() {

        // Wait _waitOneSecondA;
        Wait _waitOneSecondB;
        DeployIntake _deployIntake;
        DriveToPose _driveToA;
        // DriveAuto _driveToB;

        double _xPos; // meters
        double _yPos; // meters
        double _theta; // fractions of PI for radians
        double _omega; // fractions of PI for radians
        Double _velocity; // m/s
        
        // _waitOneSecondA = new Wait(1.0);
        _waitOneSecondB = new Wait(1.0);
        _deployIntake = new DeployIntake(_intake);

        // destination A
        _xPos = -1.0;
        _yPos = 0.0;
        _theta = 0.0;
        _omega = 1.5;
        _velocity = 0.1;

        _driveToA = getAutoDriveCommand(_xPos, _yPos, _theta, _omega, _velocity);

        // destination B
        _xPos = 0.0;
        _yPos = 0.0;
        _theta = 0.0;
        _omega = 0.5;
        _velocity = 0.2;

        // _driveToB = getAutoDriveCommand(_xPos, _yPos, _theta, _omega, _velocity);

        // These all have to be unique commands.
        // Cannot execute same command twice.
        return new SequentialCommandGroup(
            //_waitOneSecondA,
            _deployIntake,
            _driveToA,
            _waitOneSecondB
            //_driveToB
        );

    }

    /**
     * Return a drive to destination command
     * 
     * @param xPos
     * @param yPos
     * @param theta
     * @param omega
     * @param velocity
     * @return new DriveAuto
     */
    public DriveToPose getAutoDriveCommand(
        double xPos,
        double yPos,
        double theta,
        double omega,
        double velocity
        ) {

        Pose2d _wayPoint; // contains xPos, yPos and theta
        Rotation2d _heading; // uses omega
        double _theta;
        double _omega;

        _theta = theta * Math.PI;
        _omega = omega * Math.PI;

        _wayPoint = new Pose2d(xPos, yPos, new Rotation2d(_theta));
        _heading = new Rotation2d(_omega);
    
        return new DriveToPose(_driveTrain, _wayPoint, _heading, velocity);
    }

    @Override
    public void updateDashboard() {
        //Updates the driver station
        _driveTrain.updateDashboard();
        metric("AutoChooser", _autoChooser.getSelectedMode().getValue());
        _autoChooser.updateDashboard();
    }
}
