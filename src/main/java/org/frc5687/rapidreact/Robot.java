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
 * See IterativeRobotBase for the base class and methods documentation.
 * 
 * Note about overriding methods and annotations (e.g., @Override):
 * 
 * An instance method in a subclass with the same signature (name, plus the number and the type
 * of its parameters) and return type as an instance method in the superclass overrides the
 * superclass's method.
 * 
 * The ability of a subclass to override a method allows a class to inherit from a superclass whose
 * behavior is"close enough" and then to modify behavior as needed. The overriding method has the
 * same name, number and type of parameters, and return type as the method that it overrides.
 * An overriding method can also return a subtype of the type returned by the overridden method.
 * 
 * When overriding a method, you might want to use the @Override annotation that instructs the
 * compiler that you intend to override a method in the superclass. If, for some reason, the compiler
 * detects that the method does not exist in one of the superclasses, then it will generate an error.
 * 
 * Annotations, a form of metadata, provide data about a program that is not part of the program
 * itself.  Annotations have no direct effect on the operation of the code they annotate.
 * 
 * Annotations have a number of uses, among them:
 *  - Information for the compiler — Annotations can be used by the compiler to detect errors
 *    or suppress warnings.
 *  - Compile-time and deployment-time processing — Software tools can process annotation information
 *    to generate code, XML files, and so forth.
 *  - Runtime processing — Some annotations are available to be examined at runtime.
 * 
 * Note: unused variables in the class and methods below have been commented out
 * 
 */

/**
 * Robot is responsible for control flow.
 * 
 * <p>Command-based is a declarative paradigm designed to minimize the need to pay attention
 * to explicit program control flow, so the Robot class of a command-based project should
 * be mostly empty.
 * 
 * <p>The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the IterativeRobotBase documentation. If you change the name of this
 * class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class Robot extends OutliersRobot {

    // TODO: identityMode should be set in Constants file
    public static OutliersContainer.IdentityMode _identityMode =
            OutliersContainer.IdentityMode.competition;
    // TODO: log levels should be set in Constants file
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

    /* ---- Init methods called when a mode is entered --- */

    /**
     * Robot-wide initialization code
     *
     * <p>Called when the robot is first powered on. It will be called exactly one time.
     *
     * <p>Warning: the Driver Station "Robot Code" light and FMS "Robot Ready" indicators will be off
     * until RobotInit() exits. Code in RobotInit() that waits for enable will cause the robot to
     * never indicate that the code is ready, causing the robot to be bypassed in a match.
     */
    @Override
    public void robotInit() {
        loadConfigFromUSB();
        RioLogger.getInstance().init(_fileLogLevel, _dsLogLevel);
        LiveWindow.disableAllTelemetry();
        // DriverStation.silenceJoystickConnectionWarning(true);

        metric("Identity", _identityMode.toString());
        metric("Commit", Version.REVISION);
        metric("Branch", Version.BRANCH);

        info("Robot " + _name + " running in " + _identityMode.toString() + " mode");
        info("Running commit " + Version.REVISION + " of branch " + Version.BRANCH);

        _robotContainer = new RobotContainer(this, _identityMode);
        // _timer = new Timer();
        _robotContainer.init(); // allocate subsystems and set default commands

        // Periodically flushes metrics
        // TODO: configure enable/disable via USB config file
        // _time = _timer.get();
        // TODO: fix resource leak on next line (Closeable value never closed)
        new Notifier(MetricTracker::flushAll).startPeriodic(Constants.METRIC_FLUSH_PERIOD);
    }

    /**
    * Initialization code for disabled mode
    *
    * <p>Called each time the robot enters disabled mode.
     */
    @Override
    public void disabledInit() {
        // _limelight.disableLEDs();
        RioLogger.getInstance().forceSync();
        RioLogger.getInstance().close();
        _robotContainer.disabledInit();
        // MetricTracker.flushAll();
    }

    /**
     * Initialization code for autonomous mode
     *
     * <p>Called each time the robot enters autonomous mode.
     */
    @Override
    public void autonomousInit() {
        // _fmsConnected = DriverStation.isFMSAttached();
        _autoCommand = _robotContainer.getAutonomousCommand();
        _robotContainer.autonomousInit();
        if (_autoCommand != null) {
            _autoCommand.schedule();
        }
    }

    /** 
     * Initialization code for teleop mode
     *
     * <p>Called each time the robot enters teleop mode.
     * 
     * <p>It is generally good practice to have the teleopInit() method cancel
     * any still-running autonomous commands.
     */
    public void teleopInit() {
        // _fmsConnected = DriverStation.isFMSAttached();
        // Stop autonomous when teleop starts running.
        // If you want the autonomous to continue until
        // interrupted by another command, remove
        // this line or comment it out.
        if (_autoCommand != null) { _autoCommand.cancel(); }
        _robotContainer.teleopInit();
    }

    /* ----------- Periodic code called on an interval --------------- */

    /** Our own periodic method
     * 
     * <p>This is currently being called all the time by robotPeriodic().
     * 
     * <p>TODO: why create an ourPeriodic() method?
     * TODO: why does ourPeriodic() call update()?
     * 
     */
    private void ourPeriodic() {

        // Example of starting a new row of metrics for all instrumented objects.
        // MetricTracker.newMetricRowAll();
        MetricTracker.newMetricRowAll();
        _robotContainer.periodic();
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        update();
        updateDashboard();
    }

    /** Autonomous mode periodic */
    @Override
    public void autonomousPeriodic() {}

    /** Teleoperator control mode periodic */
    @Override
    public void teleopPeriodic() {}

    /** Test mode periodic */
    @Override
    public void testPeriodic() {}

    /** Disabled mode periodic */
    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
        _robotContainer.disabledPeriodic();
    }

    /**
     * robotPeriodic is called every interval, no matter the mode. Use this for items like
     * diagnostics that you want to run during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode-specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        ourPeriodic(); // runs the Scheduler
    }

    /* ---- Exit methods are called when the mode is exited --- */

    /* ----- Other methods ----- */

    /**
     * Looks like a placeholder method for periodic updates.
     * 
     * TODO: why does ourPeriodic() call update()? 
     * 
     * <p>This looks like a place to put code that should run during ourPeriodic(),
     * but why not just put that code in ourPeriodic()?
     * 
    */
    private void update() {}

    /** 
     * Update dashboard updates values on drive station and does logging.
     * 
     * Updating dashboard is expensive. Increase TICKS_PER_UPDATE to throttle back
     * how often this is done.
     * 
     */
    public void updateDashboard() {
        _updateTick++;
        if (_updateTick >= Constants.TICKS_PER_UPDATE) {
            _updateTick = 0;
            _robotContainer.updateDashboard();
        }
    }

    // TODO: loadConfigFromUSB() should probably go in util
    private void loadConfigFromUSB() {
        // String output_dir = "/U/"; // USB drive is mounted to /U on roboRIO
        try {
            String usbDir = "/U/"; // USB drive is mounted to /U on roboRIO
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
            _identityMode = OutliersContainer.IdentityMode.competition;
        }
    }

    // TODO: processConfigLine() should probably go in util
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
}
