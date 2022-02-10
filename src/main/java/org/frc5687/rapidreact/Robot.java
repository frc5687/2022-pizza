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
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
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
 */
public class Robot extends OutliersRobot implements ILoggingSource {

    // TODO: identityMode should be set in Constants file
    public static OutliersContainer.IdentityMode _identityMode =
            OutliersContainer.IdentityMode.competition;
    // TODO: log levels should be set in Constants file
    private RioLogger.LogLevel _dsLogLevel = RioLogger.LogLevel.warn;
    private RioLogger.LogLevel _fileLogLevel = RioLogger.LogLevel.warn;
    private int _updateTick = 0;
    private String _name;
    private RobotContainer _robotContainer;
    private boolean _fmsConnected;
    private Command _autoCommand;
    private Timer _timer;
    private double _prevTime;
    private double _time;

    /**
     * TODO: fix robotInit comment
     * 
     * [This comment appears to be referencing a different version of this code]
     * 
     * This function is setRollerSpeed [what does this mean?] when the robot is first started up
     * and should be used for any initialization code.
     * 
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
        _timer = new Timer();
        _robotContainer.init();

        // Periodically flushes metrics
        // TODO: configure enable/disable via USB config file
        _time = _timer.get();
        new Notifier(MetricTracker::flushAll).startPeriodic(Constants.METRIC_FLUSH_PERIOD);
    }

    /**
     * TODO: fix autonomous mode init comment
     * 
     * [The comment below appears to be referencing a different version of this method.]
     * 
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        _fmsConnected = DriverStation.isFMSAttached();
        _robotContainer.autonomousInit();
        if (_autoCommand != null) {
            _autoCommand.schedule();
        }
    }

    /** Autonomous mode periodic */
    @Override
    public void autonomousPeriodic() {}

    /** Teleoperator control mode initialization */
    public void teleopInit() {
        _fmsConnected = DriverStation.isFMSAttached();
        _robotContainer.teleopInit();

        // _limelight.disableLEDs();
    }

    /** Teleoperator control mode periodic */
    @Override
    public void teleopPeriodic() {}

    /** Our own method called periodically
     * 
     * TODO: when is Robot.ourPeriodic() called?
     * TODO: why does ourPeriodic() call update()?
     * 
     */
    private void ourPeriodic() {

        // Example of starting a new row of metrics for all instrumented objects.
        // MetricTracker.newMetricRowAll();
        MetricTracker.newMetricRowAll();
        //        _robotContainer.periodic();
        CommandScheduler.getInstance().run();
        update();
        updateDashboard();
    }

    /** TODO: What about test mode initialization? */

    /** Test mode periodic */
    @Override
    public void testPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /** Disabled mode initialization */
    @Override
    public void disabledInit() {
        // _limelight.disableLEDs();
        RioLogger.getInstance().forceSync();
        RioLogger.getInstance().close();
        _robotContainer.disabledInit();
        //        MetricTracker.flushAll();
    }

    /** Disabled mode periodic */
    @Override
    public void disabledPeriodic() {
        super.disabledPeriodic();
        _robotContainer.disabledPeriodic();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want to run during disabled, autonomous, teleoperated and test.
     *
     * This runs after the mode-specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        ourPeriodic();
    }

    /** TODO: why does ourPeriodic() call update()? 
     * 
     * This looks like a place to put code that should run during ourPeriodic(),
     * but why not just put that code in ourPeriodic()?
     * 
    */
    private void update() {}

    // Updating dashboard is expensive. Increase TICKS_PER_UPDATE
    // to throttle back how often this is done.
    public void updateDashboard() {
        _updateTick++;
        if (_updateTick >= Constants.TICKS_PER_UPDATE) {
            _updateTick = 0;
            _robotContainer.updateDashboard();
        }
    }

    // TODO: loadConfigFromUSB() should probably go in util
    private void loadConfigFromUSB() {
        String output_dir = "/U/"; // USB drive is mounted to /U on roboRIO
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
