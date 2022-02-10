/* Team 5687 (C)2021 */
package org.frc5687.rapidreact;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.frc5687.rapidreact.commands.Drive;
import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.OutliersSubsystem;
import org.frc5687.rapidreact.util.OutliersContainer;

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

    public RobotContainer(Robot robot, IdentityMode identityMode) {
        super(identityMode);
        _robot = robot;
    }

    public void init() {
        _oi = new OI();
        //Config the NavX
        _imu = new AHRS(SPI.Port.kMXP, (byte) 200);
        _driveTrain = new DriveTrain(this, _oi, _imu);
        //The robots default command will run so long as another command isn't activated
        setDefaultCommand(_driveTrain, new Drive(_driveTrain, _oi));
        _robot.addPeriodic(this::controllerPeriodic, 0.005, 0.005);
        _imu.reset();
        _oi.initializeButtons(_driveTrain);
    }

    public void periodic() {
        //Runs every 20ms
    }

    public void disabledPeriodic() {
        //Runs every 20ms during disabled
    }

    @Override
    public void disabledInit() {
        //Runs once during disabled
    }

    @Override
    public void teleopInit() {
        //Runs at the start of teleop
    }

    @Override
    public void autonomousInit() {
        //This is where autos go
        //Runs once during auto
    }

    private void setDefaultCommand(OutliersSubsystem subSystem, OutliersCommand command) {
        if (subSystem == null || command == null) {
            return;
        }
        CommandScheduler s = CommandScheduler.getInstance();
        s.setDefaultCommand(subSystem, command);
    }

    @Override
    public void updateDashboard() {
        //Updates the driver station
        _driveTrain.updateDashboard();
    }

    public void controllerPeriodic() {
        if (_driveTrain != null) {
            _driveTrain.controllerPeriodic();
        }
    }
}
