/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.rapidreact.commands.DriveOI;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.commands.auto.ZeroBallAuto;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.util.AutoChooser;
import org.frc5687.rapidreact.util.OutliersContainer;

/**
 * TODO: explain RobotContainer class
 */
public class RobotContainer extends OutliersContainer {

    private OI _oi;
    private AutoChooser _autoChooser;
    private AHRS _imu;

    private Robot _robot;
    private DriveTrain _driveTrain;

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
        metric("Selected Path", "Mode: " + _autoChooser.getSelectedMode().getLabel() + ", Position: " + _autoChooser.getSelectedPosition().getLabel());

        info("Running RobotContainer.init()");

        // DriveTrain's default command is DriveOI
        setDefaultCommand(_driveTrain, new DriveOI(_driveTrain, _oi));
        // Run the control loop for each individual swerve drive unit every 5 ms.
        // DriveTrain has four DiffSwerveModules.
        // controllerPeriodic calls the periodic for each of them.
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
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

    public Command getAutonomousCommand() {
        // AutoChooser.Mode autoMode = _autoChooser.getSelectedMode();
        // AutoChooser.Position autoPosition = _autoChooser.getSelectedPosition();
        //if(autoMode.label == "ZB") {
        return new ZeroBallAuto(_driveTrain);
        // }        
    }

    @Override
    public void updateDashboard() {
        //Updates the driver station
        _driveTrain.updateDashboard();
        metric("AutoChooser", _autoChooser.getSelectedMode().getValue());
        _autoChooser.updateDashboard();
    }
}
