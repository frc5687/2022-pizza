/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.rapidreact.config.Constants;

// import org.frc5687.rapidreact.util.AutoChooser;
import org.frc5687.rapidreact.util.OutliersContainer;

import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Catapult;

import org.frc5687.rapidreact.commands.DriveOI;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.commands.auto.DeployIntake;
import org.frc5687.rapidreact.commands.auto.DriveToPose;
import org.frc5687.rapidreact.commands.auto.OneBallAuto;
import org.frc5687.rapidreact.commands.auto.Wait;
import org.frc5687.rapidreact.commands.auto.ZeroBallAuto;

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
    // private AutoChooser _autoChooser;
    private AHRS _imu;

    public Catapult catapult;
    public DriveTrain driveTrain;
    private Indexer indexer;
    public Intake intake;

    /** Create RobotContainer 
     * 
     * @param robot this robot
     * @param identityMode TODO: document identityMode
     */
    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    // Initialization methods

    /** Run once when robot code starts. */
    public void init() {
        info("Running RobotContainer.init()");

        _oi = new OI();
        // _autoChooser = new AutoChooser();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200); //Config the NavX

        // Create subsystems
        driveTrain = new DriveTrain(this, _oi, _imu);
        catapult = new Catapult(this);
        indexer = new Indexer(this);
        intake = new Intake(this);

        _oi.initializeButtons(this);

        // What command to run if nothing else is scheduled for driveTrain
        setDefaultCommand(driveTrain, new DriveOI(driveTrain, _oi));

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
        indexer.setState(Indexer.IndexerState.DEPLOYED);
        intake.setState(Intake.IntakeState.STOWED);
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

    /** Wrap DriveTrain.controllerPeriodic.
     * 
     * <p> _driveTrain can be null during initial development
    */
    public void controllerPeriodic() {
        if (driveTrain != null) {
            driveTrain.controllerPeriodic();
        }
    }

    /** Wrap CommandScheduler.setDefaultCommand.
     * 
     * <p> Allow nulls during initial development.
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

    /** Return sequence of commands to run during auto
     * 
     * @return SequentialCommandGroup
    */
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













        WaitCommand _waitOneSecondA;
        WaitCommand _waitOneSecondB;
        DeployIntake _deployIntake;
        DriveToPose _driveToA;
        // DriveAuto _driveToB;

        double _xPos; // meters
        double _yPos; // meters
        double _theta; // fractions of PI for radians
        double _omega; // fractions of PI for radians
        Double _velocity; // m/s
        
        // _waitOneSecondA = new WaitCommand(1.0);
        _waitOneSecondB = new WaitCommand(1.0);
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

        // _driveToB = getAutoDriveCommand(_xPos, _yPos, _theta, _omega, _velocity);
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

        return _auto;

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

        _theta = theta/* Math.PI*/;
        _omega = omega/* Math.PI*/;

        _wayPoint = new Pose2d(xPos, yPos, new Rotation2d(_theta));
        _heading = new Rotation2d(_omega);
    
        return new DriveToPose(_driveTrain, _wayPoint, _heading, velocity);
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
