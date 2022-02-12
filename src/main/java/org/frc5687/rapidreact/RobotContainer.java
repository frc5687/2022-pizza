/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.rapidreact.commands.Drive;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.util.OutliersContainer;
import org.frc5687.rapidreact.Lights;
/** 
 * RobotContainer is where most of the setup for your command-based robot will take place.
 * Define your robot subsystems and commands, bind those commands to triggering events
 * (such as buttons), and specify which command you will run in your autonomous routine.
 * 
 * <p>Subsystems are declared as private fields, aligned with object-oriented best-practices.
 * This makes the control flow of the program easier to keep track of as it is immediately
 * obvious which parts of the code can change or be changed by which other parts of the code.
 * This also allows the resource-management system to do its job, as users are unable to
 * make conflicting calls to subsystem methods outside of the resource-managed commands.
*/
public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AHRS _imu;

    private Robot _robot;
    private DriveTrain _driveTrain;
    private Lights _lights;
    private Command _autoCommand;

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    /* ---- Init methods called when a mode is entered --- */

    /** 
     * Initialize RobotContainer
     * 
     * <p>Called by Robot.robotInit() when robot is first powered on.
     * 
     * <p>Allocate subsystems and set default commands.
     */
    public void init() {
         // 2020 code says OI must be first...
        _oi = new OI();
        // Config the NavX
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _imu.reset(); // 2020 code calls zeroYaw()

        _driveTrain = new DriveTrain(this, _oi, _imu);
        _lights = new Lights(RobotMap.PWM.BLINKIN, Constants.Blinkins.BLINKIN_LENGTH);
        // Initialize buttons AFTER subsystems allocated
        _oi.initializeButtons(_driveTrain);

        // The robot's default command will run so long as another command isn't activated
        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        final double PERIOD_SECONDS = 0.005;
        final double OFFSET_SECONDS = 0.005;
        _robot.addPeriodic(this::controllerPeriodic, PERIOD_SECONDS, OFFSET_SECONDS);
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void disabledInit() {}

    @Override
    public void teleopInit() {}

    /* ---- Periodic methods ---- */

    public void periodic() {
        // Code from 2020 robot polls for KillAll
        /*
        _oi.poll();
        if (_oi.isKillAllPressed()) {
            new KillAll(_driveTrain, _shooter, _indexer, _intake, _turret, _hood).schedule();
            _indexer.stopAgitator();
        }
        */
        _lights.setRed();
    }

    public void disabledPeriodic() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return _autoCommand;

        // Code from 2020 robot for autochooser
        /*
        AutoChooser.Mode autoMode = _autoChooser.getSelectedMode();

        switch (autoMode) {
            case ShootAndGo:
                return wrapCommand(new AutoShootAndGo(_turret, _shooter, _hood, _limelight, _driveTrain, _intake, _poseTracker, _indexer, _lights));
            case ShootAndNearTrench:
                return wrapCommand(new AutoShootAndNearTrench(_turret, _shooter, _hood, _limelight, _driveTrain, _poseTracker, _indexer, _intake, _lights));
            case ShootAndFarTrench:
                return wrapCommand(new AutoShootAndFarTrench(_turret, _shooter, _hood, _limelight, _driveTrain, _poseTracker, _indexer, _intake, _lights));
            case Generator2NearTrench:
                return wrapCommand(new EightBallAuto(_driveTrain, _turret, _shooter,_hood,_intake, _imu, _indexer,_lights, _limelight, _poseTracker));
            default:
                return new SequentialCommandGroup(
                        new ZeroSensors(_hood, _turret),
                        new AutoShootAndGo(_turret, _shooter, _hood, _limelight, _driveTrain, _intake, _poseTracker, _indexer, _lights)
                //      new EightBallAuto(_driveTrain, _turret, _shooter,_hood,_intake, _imu, _indexer,_lights, _limelight, _poseTracker)
                );
        }
          
        */
    }

    /**
     * Helper function to wrap CommandScheduler.setDefaultCommand.
     * Allows us to pass nulls during initial development without breaking.
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

    /**
     * Helper function to wrap Drivetrain.controllerPeriodic.
     * 
     */
    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.controllerPeriodic();
        }
    }

    @Override
    public void updateDashboard() {
        //Updates the driver station
        super.updateDashboard();
        _driveTrain.updateDashboard();
    }

}
