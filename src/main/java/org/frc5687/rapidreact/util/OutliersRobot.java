/* (C)2021 */
package org.frc5687.rapidreact.util;

import org.frc5687.rapidreact.config.Constants;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Robot extends this abstract class.
 * 
 * TODO: why do we create this abstract class?
 * 
 */
public abstract class OutliersRobot extends TimedRobot implements ILoggingSource {

    /** Create an OutliersRobot */
    public OutliersRobot() {
        super(Constants.UPDATE_PERIOD);
    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
    }

    @Override
    public void error(String message) {
        RioLogger.error(this, message);
    }

    @Override
    public void warn(String message) {
        RioLogger.warn(this, message);
    }

    @Override
    public void info(String message) {
        RioLogger.info(this, message);
    }

    @Override
    public void debug(String message) {
        RioLogger.debug(this, message);
    }
}
