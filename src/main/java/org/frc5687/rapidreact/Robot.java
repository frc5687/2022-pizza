/* Team 5687 (C)2020-2022 */
package org.frc5687.rapidreact;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import org.frc5687.rapidreact.util.*;

/**
 * The VM is configured to automatically run Robot, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 * 
 * <p>RobotContainer has the actual commands.
 * 
 * <p>Robot can be in four modes: disabled, autonomous, teleop, or test.
 */
public class Robot extends OutliersRobot {

    public static OutliersContainer.IdentityMode _identityMode =
            OutliersContainer.IdentityMode.programming;
    private RioLogger.LogLevel _dsLogLevel = RioLogger.LogLevel.warn;
    private RioLogger.LogLevel _fileLogLevel = RioLogger.LogLevel.warn;

    private int _updateTick = 0;

    private String _name;

    private RobotContainer _robotContainer;

    // private boolean _fmsConnected;

    private Command _autoCommand;

    // private Timer _timer;
    // private double _prevTime;
    // private double _time;

    // Initialization methods

    // Initialization methods

    /**
     * TODO: Explain robotInit
     */
    @Override
    public void robotInit() {
        loadConfigFromUSB();
        RioLogger.getInstance().init(_fileLogLevel, _dsLogLevel);
        LiveWindow.disableAllTelemetry();
        DriverStation.silenceJoystickConnectionWarning(true);

        metric("Identity", _identityMode.toString());
        metric("Commit", Version.REVISION);
        metric("Branch", Version.BRANCH);

        info("Robot " + _name + " running in " + _identityMode.toString() + " mode");
        info("Running commit " + Version.REVISION + " of branch " + Version.BRANCH);

        _robotContainer = new RobotContainer(this, _identityMode);
        // _timer = new Timer();
        _robotContainer.init();
        _autoCommand = _robotContainer.getAutonomousCommand();

        // Periodically flushes metrics
        // TODO: configure enable/disable via USB config file
        // _time = _timer.get();
        new Notifier(MetricTracker::flushAll).startPeriodic(Constants.METRIC_FLUSH_PERIOD);
    }

    @Override
    public void disabledInit() {
        // _limelight.disableLEDs();
        RioLogger.getInstance().forceSync();
        RioLogger.getInstance().close();
        _robotContainer.disabledInit();
        // MetricTracker.flushAll();
    }

    /**
     * TODO: Explain autonomousInit
     */
    @Override
    public void autonomousInit() {
        // _fmsConnected = DriverStation.isFMSAttached();
        _robotContainer.autonomousInit();
        if (_autoCommand != null) {
            _autoCommand.schedule();
        }
    }

    public void teleopInit() {
        // _fmsConnected = DriverStation.isFMSAttached();
        _robotContainer.teleopInit();

        // _limelight.disableLEDs();
    }

    // Periodic methods

    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
        _robotContainer.disabledPeriodic();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        ourPeriodic();
    }

    private void ourPeriodic() {

        // Example of starting a new row of metrics for all instrumented objects.
        // MetricTracker.newMetricRowAll();
        MetricTracker.newMetricRowAll();
        // _robotContainer.periodic();
        CommandScheduler.getInstance().run();
        update();
        updateDashboard();
    }

    // Helper methods

    public void updateDashboard() {
        _updateTick++;
        if (_updateTick >= Constants.TICKS_PER_UPDATE) {
            _updateTick = 0;
            _robotContainer.updateDashboard();
        }
    }

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
            _identityMode = OutliersContainer.IdentityMode.programming;
        }
    }

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

    private void update() {}
}
