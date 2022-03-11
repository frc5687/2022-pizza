/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import org.frc5687.rapidreact.config.Constants;
import org.frc5687.rapidreact.util.*;

/**
 * The VM is configured to automatically run Robot, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, update the build.gradle file.
 * 
 * <p>Responsible for the main control flow of the robot code.
 * 
 * <p>Why separate Robot and RobotContainer classes?
 * See https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html
 * 
 * <p>Robot can be in four modes: disabled, autonomous, teleop, or test.
 */
public class Robot extends OutliersRobot {

    private String _name = Constants.ROBOT_NAME;
    public static OutliersContainer.IdentityMode _identityMode = Constants.IDENTITY;
    private RioLogger.LogLevel _dsLogLevel = Constants.DS_LOG_LVL;
    private RioLogger.LogLevel _fileLogLevel = Constants.USB_LOG_LVL;

    private int _updateTick = 0;

    private RobotContainer _robotContainer;

    // private boolean _fmsConnected;

    private Command _autoCommand;

    // Initialization methods

    /** Run once when robot code starts.
     * 
     * <p>Set up logging and whatever else needs to happen only once when the robot code starts.
     * 
     * <p>NB: Do NOT get autonomous command from RobotContainer in this method.  Do that in
     * autonomousInit().  That allows us to have a rotary selector dial and choose different
     * auto routines to run.
     */
    @Override
    public void robotInit() {
        loadConfigFromUSB();
        info("Running Robot.robotInit()");

        RioLogger.getInstance().init(_fileLogLevel, _dsLogLevel);
        LiveWindow.disableAllTelemetry();
        DriverStation.silenceJoystickConnectionWarning(true);

        metric("Identity", _identityMode.toString());
        metric("Commit", Version.REVISION);
        metric("Branch", Version.BRANCH);

        info("Robot " + _name + " running in " + _identityMode.toString() + " mode");
        info("Running commit " + Version.REVISION + " of branch " + Version.BRANCH);

        _robotContainer = new RobotContainer(this, _identityMode);
        _robotContainer.init();

        // Periodically flush metrics
        // TODO: configure enable/disable via USB config file
        new Notifier(MetricTracker::flushAll).startPeriodic(Constants.METRIC_FLUSH_PERIOD);
    }

    @Override
    public void disabledInit() {
        info("Running Robot.disabledInit()");

        // _limelight.disableLEDs();
        RioLogger.getInstance().forceSync();
        RioLogger.getInstance().close();
        _robotContainer.disabledInit();
        // MetricTracker.flushAll();
    }

    /** Run once when robot enters autonomous mode
     * 
     * <p>Schedule the auto command.
     */
    @Override
    public void autonomousInit() {
        info("Running Robot.autonomousInit()");

        // Get auto command when we enter autonomous mode.
        // Setting it here rather than in robotInit() allows us to turn on our robot,
        // then use a manual rotary selector to choose which autocommand to run.
        _autoCommand = _robotContainer.getAutonomousCommand();

        // _fmsConnected = DriverStation.isFMSAttached();
        _robotContainer.autonomousInit();
        if (_autoCommand != null) {
            _autoCommand.schedule();
        }
    }

    public void teleopInit() {
        info("Running Robot.teleopInit()");

        // Good practice to cancel the autonomous command that may still be running.
        // If you want the autonomous to continue until interrupted by another command,
        // comment the following out.
        if (_autoCommand != null) {
            _autoCommand.cancel();
        }

        // _fmsConnected = DriverStation.isFMSAttached();
        _robotContainer.teleopInit();
    }

    // Periodic methods

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
        _robotContainer.disabledPeriodic();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testPeriodic() {}

    /** Call every robot cycle, no matter the mode.
     * 
     * <p>Use for items like diagnostics to run during disabled, autonomous, teleoperated
     * and test.
     *
     * <p>This runs after the mode-specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        ourPeriodic();
    }

    /** What we want to run periodically.
     * 
     * <p>Write to log, schedule commands and update the dashboard.
     */
    private void ourPeriodic() {

        // Example of starting a new row of metrics for all instrumented objects.
        // MetricTracker.newMetricRowAll();
        MetricTracker.newMetricRowAll();

        _robotContainer.periodic();

        // NB: It is essential to run the scheduler each cycle!  Otherwise nothing happens.
        CommandScheduler.getInstance().run();

        update();
        updateDashboard();
    }

    // Helper methods

    /** Load configuration file from USB thumb drive attached to roboRio. */
    private void loadConfigFromUSB() {
        String output_dir = "/U/"; // USB drive is mounted to /U on roboRIO

        try {
            String usbDir = output_dir;
            String configFileName = usbDir + "frc5687.cfg";
            File configFile = new File(configFileName);
            FileReader reader = new FileReader(configFile);
            BufferedReader bufferedReader = new BufferedReader(reader);

            String line;
            while ((line = bufferedReader.readLine()) != null) {
                processConfigLine(line);
            }

            bufferedReader.close();
            reader.close();
        } catch (Exception e) {
            // in case USB config isn't found
            _identityMode = OutliersContainer.IdentityMode.programming;
        }
    }

    /** Parse one line of the configuration file */
    private void processConfigLine(String line) {
        try {
            if (line.startsWith("#")) {
                return;
            }
            String[] a = line.split("=");
            if (a.length == 2) {
                String key = a[0].trim().toLowerCase();
                String value = a[1].trim();
                switch (key) {
                    case "name":
                        _name = value;
                        break;
                    case "mode":
                        _identityMode = OutliersContainer.IdentityMode.valueOf(value.toLowerCase());
                        break;
                    case "fileloglevel":
                        _fileLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        break;
                    case "dsloglevel":
                        _dsLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        break;
                }
            }
        } catch (Exception e) {

        }
    }

    /** Updates to run every cycle before updating dashboard. */
    private void update() {}

    /** Update the dashboard.
     * 
     * <p>Last thing we do every cycle.
     * 
     * <p>Whether we actually update every cycle depends on TICKS_PER_UPDATE.
     * Updating every tick can slow down the robot.
     */
    public void updateDashboard() {
        _updateTick++;
        if (_updateTick >= Constants.TICKS_PER_UPDATE) {
            _updateTick = 0;
            //TODO: Uncomment
            //_robotContainer.updateDashboard();
        }
    }
}
