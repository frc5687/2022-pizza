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
import org.frc5687.rapidreact.commands.auto.DeployIntake;
import org.frc5687.rapidreact.commands.auto.OneBallAuto;
import org.frc5687.rapidreact.commands.auto.Wait;
import org.frc5687.rapidreact.commands.auto.ZeroBallAuto;

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
        Pose2d spot3 = new Pose2d(6.9, 2.5, new Rotation2d());
        
        _driveTrain.resetOdometry(spot3);

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

        AutoChooser.Position autoPosition = _autoChooser.getSelectedPosition();
        AutoChooser.Mode autoMode = _autoChooser.getSelectedMode();
        

        switch(autoPosition) {
            case First:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.FIRST);
                switch(autoMode) {
                    case ZeroBall:
                        return new ZeroBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_ONE);
                    case OneBall:
                        return new OneBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_ONE);
                }
            case Second:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.SECOND);
                switch(autoMode) {
                    case ZeroBall:
                    case OneBall:
                }
            case Third:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.THIRD);
                switch(autoMode) {
                    case ZeroBall:
                        return new ZeroBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_TWO);
                    case OneBall:
                }       return new OneBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_TWO);
            case Fourth:
                _driveTrain.resetOdometry(Constants.Auto.RobotPositions.FOURTH);
                switch(autoMode) {
                    case ZeroBall:
                        return new ZeroBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_THREE);
                    case OneBall:
                        return new OneBallAuto(_driveTrain, Constants.Auto.BallPositions.BALL_THREE);
                }
        }













        Wait _waitOneSecondA;
        Wait _waitOneSecondB;
        DeployIntake _deployIntake;
        DriveAuto _driveToA;
        DriveAuto _driveToB;

        double _xPos;
        double _yPos;
        double _theta; // fractions of PI for radians
        double _omega; // fractions of PI for radians
        Double _velocity;
        
        _waitOneSecondA = new Wait(1.0);
        _waitOneSecondB = new Wait(1.0);
        _deployIntake = new DeployIntake(_intake);

        //These are the balls' exact positions,
        //be careful not to use ball3's position, or you'll run into the wall
        Pose2d ball1 = new Pose2d(4.8, 6.2, new Rotation2d());
        Pose2d ball2 = new Pose2d(5.1, 1.77, new Rotation2d());
        Pose2d ball3 = new Pose2d(7.7, 0.28, new Rotation2d());

        // destination A
        _xPos = ball2.getX();
        _yPos = ball2.getY();
        _theta = 0.69;
        _omega = 0.0;
        _velocity = 0.1;

        _driveToA = getAutoDriveCommand(_xPos, _yPos, _theta, _omega, _velocity);

        // destination B
        _xPos = 2.0;
        _yPos = 2.0;
        _theta = 0.69;
        _omega = 0.0;
        _velocity = 0.2;

        _driveToB = getAutoDriveCommand(_xPos, _yPos, _theta, _omega, _velocity);
        return null;
        // These all have to be unique commands.
        // Cannot execute same command twice.
        // return new SequentialCommandGroup(
        //     //_waitOneSecondA,
        //     _deployIntake,
        //     _driveToA,
        //     //_waitOneSecondB
        //     _driveToB
        // );

    }

    /** Return a drive to destination command */
    public DriveAuto getAutoDriveCommand(
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

        _theta = theta/* Math.PI*/;
        _omega = omega/* Math.PI*/;

        _wayPoint = new Pose2d(xPos, yPos, new Rotation2d(_theta));
        _heading = new Rotation2d(_omega);
    
        return new DriveAuto(_driveTrain, _wayPoint, _heading, velocity);
    }

    @Override
    public void updateDashboard() {
        //Updates the driver station
        _driveTrain.updateDashboard();
        // metric("AutoChooser", _autoChooser.getSelectedMode().getValue());
        _autoChooser.updateDashboard();
        metric("Position", _autoChooser.getSelectedPosition().name());
        metric("Mode", _autoChooser.getSelectedMode().name());
    }
}
