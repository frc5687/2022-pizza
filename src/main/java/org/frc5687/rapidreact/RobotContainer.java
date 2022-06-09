/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import java.io.IOException;
import java.net.SocketException;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.frc5687.rapidreact.config.Constants;

import org.frc5687.rapidreact.util.AutoChooser;
import org.frc5687.rapidreact.util.JetsonProxy;
import org.frc5687.rapidreact.util.OutliersContainer;

import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Maverick;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DiffIntake;
import org.frc5687.rapidreact.commands.Drive;
import org.frc5687.rapidreact.commands.OutliersCommand;
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

    private AutoChooser autoChooser;
    AutoChooser.Position autoPosition;
    AutoChooser.Mode autoMode;

    private OI _oi;

    private AHRS _imu;

    public Catapult catapult;
    public DriveTrain driveTrain;
    private Indexer indexer;
    public Intake intake;
    public Maverick maverick;

    private DiffIntake diffIntake;

    private JetsonProxy _proxy;

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
    public void init(){
        info("Running RobotContainer.init()");

        // Display starting position value
        SmartDashboard.putString("DB/String 0", "Starting Position:");
        SmartDashboard.putString("DB/String 5", "Unknown");

        // Display auto mode value
        SmartDashboard.putString("DB/String 1", "Auto Mode:");
        String _automodeString = SmartDashboard.getString("Auto Selector", "Unknown");
        SmartDashboard.putString("DB/String 6", _automodeString);

        // Auto mode chooser
        String [] modes = { "Zero Ball", "One Ball", "Two Ball", "Three Ball", "Four Ball", "Five Ball" };
        SmartDashboard.putStringArray("Auto List", modes);

        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200); //Config the NavX
        _proxy = new JetsonProxy();
        _proxy.run();

        diffIntake = new DiffIntake(this);

        // Create subsystems
        driveTrain = new DriveTrain(this, _oi, _imu);
        catapult = new Catapult(this);
        indexer = new Indexer(this);
        intake = new Intake(this);
        maverick = new Maverick(this, driveTrain);
        maverick.stopRumble();

        _oi.initializeButtons(this, maverick, driveTrain, diffIntake);

        // What command to run if nothing else is scheduled for driveTrain

        // Run the control loop for each individual swerve drive unit every 5 ms.
        // DriveTrain has four DiffSwerveModules.
        // controllerPeriodic calls the periodic for each of them.
        _robot.addPeriodic(this::controllerPeriodic, Constants.DifferentialSwerveModule.kDt, 0.005);

        _imu.reset();
        setDefaultCommand(driveTrain, new Drive(driveTrain, _oi));
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

        // Check starting position buttons
        Boolean _positionOne = SmartDashboard.getBoolean("DB/Button 0", false);
        Boolean _positionTwo = SmartDashboard.getBoolean("DB/Button 1", false);
        Boolean _positionThree = SmartDashboard.getBoolean("DB/Button 2", false);
        Boolean _positionFour = SmartDashboard.getBoolean("DB/Button 3", false);

        // Set position to highest value that is selected
        autoPosition = AutoChooser.Position.Unknown;
        if (_positionOne) {
            autoPosition = AutoChooser.Position.First;
            SmartDashboard.putString("DB/String 5", "One");
        }
        if (_positionTwo) {
            autoPosition = AutoChooser.Position.Second;
            SmartDashboard.putString("DB/String 5", "Two");
        }
        if (_positionThree) {
            autoPosition = AutoChooser.Position.Third;
            SmartDashboard.putString("DB/String 5", "Three");
        }
        if (_positionFour) {
            autoPosition = AutoChooser.Position.Fourth;
            SmartDashboard.putString("DB/String 5", "Four");
        }
        if (autoPosition == AutoChooser.Position.Unknown) {
            SmartDashboard.putString("DB/String 5", "Unknown");
        }

        // Display auto mode selector
        String _automodeString = SmartDashboard.getString("Auto Selector", "Unknown");
        SmartDashboard.putString("DB/String 6", _automodeString);

        updateDashboard();
    }

    // Helper methods

    /** Wrap DriveTrain.controllerPeriodic.
     * 
     * <p> driveTrain can be null during initial development
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

        String _automode = SmartDashboard.getString("Auto Selector", "Zero Ball");
        
        switch(_automode) {
            case "Zero Ball":
                autoMode = AutoChooser.Mode.ZeroBall;
                break;
            case "One Ball":
                autoMode = AutoChooser.Mode.OneBall;
                break;
            case "Two Ball":
            case "Three Ball":
            case "Four Ball":
            case "Five Ball":
            default:
                autoMode = AutoChooser.Mode.Unknown;
        }

        // AutoChooser.Position autoPosition = _autoChooser.getSelectedPosition();
        // AutoChooser.Mode autoMode = _autoChooser.getSelectedMode();

        // autoPosition = AutoChooser.Position.Fourth;
        // autoMode = AutoChooser.Mode.OneBall;

        Pose2d[] destinationsZeroBall = { new Pose2d() };
        Pose2d[] destinationsOneBall = { new Pose2d() };
        Rotation2d[] rotationsZeroBall = { new Rotation2d() };
        Rotation2d[] rotationsOneBall = { new Rotation2d() };
        
        switch(autoPosition) {
            case First:
                driveTrain.resetOdometry(Constants.Auto.RobotPositions.FIRST);
                destinationsZeroBall[0] = Constants.Auto.BallPositions.BALL_ONE;
                destinationsOneBall[0] = Constants.Auto.BallPositions.BALL_ONE;
                break;
            case Second:
                driveTrain.resetOdometry(Constants.Auto.RobotPositions.SECOND);
                destinationsZeroBall[0] = Constants.Auto.FieldPositions.ROBOT_POS_TWO_DEST;
                destinationsOneBall[0] = Constants.Auto.FieldPositions.ROBOT_POS_TWO_DEST;
                break;
            case Third:
                driveTrain.resetOdometry(Constants.Auto.RobotPositions.THIRD);
                destinationsZeroBall[0] = Constants.Auto.BallPositions.BALL_TWO;
                destinationsOneBall[0] = Constants.Auto.BallPositions.BALL_TWO;
                break;
            case Fourth:
                driveTrain.resetOdometry(Constants.Auto.RobotPositions.FOURTH);
                destinationsZeroBall[0] = Constants.Auto.FieldPositions.SAFE_BALL_THREE;
                destinationsOneBall[0] = Constants.Auto.FieldPositions.SAFE_BALL_THREE;
                rotationsZeroBall[0] = Constants.Auto.Rotations.BALL_THREE_FROM_FOURTH;
                rotationsOneBall[0] = Constants.Auto.Rotations.BALL_THREE_FROM_FOURTH;
                break;
            default:
                return new Wait(15);
        }

        switch (autoMode) {
            case ZeroBall:
                return new ZeroBallAuto(driveTrain, destinationsZeroBall[0], rotationsZeroBall[0]);
            case OneBall:
                return new OneBallAuto(driveTrain, destinationsOneBall[0], rotationsOneBall[0]);
            default:
                return new Wait(15);
        }

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
    // public DriveToPose getAutoDriveCommand(
    //     double xPos,
    //     double yPos,
    //     double theta,
    //     double omega,
    //     double velocity
    //     ) {

    //     Pose2d _wayPoint; // contains xPos, yPos and theta
    //     Rotation2d _heading; // uses omega
    //     double _theta;
    //     double _omega;

    //     _theta = theta/* Math.PI*/;
    //     _omega = omega/* Math.PI*/;

    //     _wayPoint = new Pose2d(xPos, yPos, new Rotation2d(_theta));
    //     _heading = new Rotation2d(_omega);
    
    //     return new DriveToPose(driveTrain, xPos, yPos, _theta, _omega, velocity);
    // }

    @Override
    public void updateDashboard() {
        // Updates the driver station
        if (driveTrain != null) {
            driveTrain.updateDashboard();
        }
        if (autoChooser != null) {
            autoChooser.updateDashboard();
        }
        if (autoPosition != null) {
            metric("Auto Position", autoPosition.name());
        }
        if (autoMode != null) {
            metric("Auto Mode", autoMode.name());
        }

        maverick.updateDashboard();
    }
}
