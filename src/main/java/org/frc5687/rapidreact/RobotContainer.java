/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.rapidreact.commands.DriveOI;
// import org.frc5687.rapidreact.commands.DriveTrajectory;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.commands.auto.ZeroBallAuto;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.util.AutoChooser;
import org.frc5687.rapidreact.util.OutliersContainer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

import java.io.IOException;
import java.nio.file.Path;
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
        //Config the NavX
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _driveTrain = new DriveTrain(this, _oi, _imu);
        metric("Selected Path", "Mode: " + _autoChooser.getSelectedMode().getLabel() + ", Position: " + _autoChooser.getSelectedPosition().getLabel());
        //Trajectory zeroBall = getTrajectory(_autoChooser.getPath(_autoChooser.getSelectedMode(), _autoChooser.getSelectedPosition()));
        //Trajectory zeroBall = getTrajectory("output/ZBLeft1ballT.wpilib.json");
        //metric("initial", zeroBall.getInitialPose().toString());

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
        //This is where autos go
        //Runs once during auto
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

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.controllerPeriodic();
        }
    }

    // Helper methods

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

    private Trajectory getTrajectory(String trajectoryJSON) {
        Trajectory trajectory = null;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            error("Trajectory init pose is " + trajectory.getInitialPose().toString());
            //            trajectory = trajectory.transformBy(transform);
            error("Trajectory successfully opened.");
        } catch (IOException ex) {
            error("Unable to open trajectory: " + trajectoryJSON + ex.getMessage());
        }
        return trajectory;
    }

    public Command getAutonomousCommand() {
        // AutoChooser.Mode autoMode = _autoChooser.getSelectedMode();
        // AutoChooser.Position autoPosition = _autoChooser.getSelectedPosition();
        //Trajectory trajectory = getTrajectory(_autoChooser.getPath(autoMode, autoPosition));
        Trajectory trajectory = getTrajectory("output/ZBLeft1ballT.wpilib.json");
        //if(autoMode.label == "ZB") {
        return new ZeroBallAuto(_driveTrain, trajectory);
        //}
        
    }

    @Override
    public void updateDashboard() {
        //Updates the driver station
        _driveTrain.updateDashboard();
        metric("AutoChooser", _autoChooser.getSelectedMode().getValue());
        _autoChooser.updateDashboard();
    }
}
