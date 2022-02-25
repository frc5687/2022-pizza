/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;

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
import org.frc5687.rapidreact.subsystems.Catapult;

import org.frc5687.rapidreact.commands.DriveOI;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.commands.auto.AutoZeroBall;
import org.frc5687.rapidreact.commands.auto.AutoOneBall;

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

    public DriveTrain driveTrain;
    private Indexer _indexer;
    public Intake intake;
    public Catapult catapult;

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
        driveTrain = new DriveTrain(this, _oi, _imu);
        _indexer = new Indexer(this);
        intake = new Intake(this);
        catapult = new Catapult(this);

        _oi.initializeButtons(driveTrain, _indexer, intake);

        // DriveTrain's default command is DriveOI
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
        _indexer.setState(Indexer.IndexerState.DEPLOYED);
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

    /** 
     * Helper function to wrap DriveTrain.controllerPeriodic.  
     * _driveTrain can be null during initial development
    */
    public void controllerPeriodic() {
        if (driveTrain != null) {
            driveTrain.controllerPeriodic();
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

        SequentialCommandGroup _auto;
        
        // TODO: replace this with AutoChooser once we have a switch

        // Choose which auto to run
        switch(Constants.Auto.AUTO_MODE) {
            case ZERO_BALL:
                _auto = new AutoZeroBall(this);
                break;
            case ONE_BALL:
                _auto = new AutoOneBall(this);
                break;
            default:
                _auto = new AutoZeroBall(this);
        }

        return _auto;

    }

    @Override
    public void updateDashboard() {
        //Updates the driver station
        driveTrain.updateDashboard();
        metric("AutoChooser", _autoChooser.getSelectedMode().getValue());
        _autoChooser.updateDashboard();
    }
}
