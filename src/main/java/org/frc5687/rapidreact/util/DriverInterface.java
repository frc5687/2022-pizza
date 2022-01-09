package org.frc5687.rapidreact.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriverInterface {
    
    public DriverInterface(){
        
    }

    public void warn(String error){
        try{
            DriverStation.reportWarning(error, false);
        }catch(Exception e){
            DriverStation.reportWarning(e.toString(), false);
        }
    }

    public void warn(Double error){
        try{
            DriverStation.reportWarning(error.toString(), false);
        }catch(Exception e){
            DriverStation.reportWarning(e.toString(), false);
        }
    }

    public void warn(int error){
        try{
            String errorString = Integer.toString(error);
            DriverStation.reportWarning(errorString, false);
        }catch(Exception e){
            DriverStation.reportWarning(e.toString(), false);
        }
    }

    public void warn(boolean error){
        try{
            String errorString = Boolean.toString(error);
            DriverStation.reportWarning(errorString, false);
        }catch(Exception e){
            DriverStation.reportWarning(e.toString(), false);
        }
    }

    public void error(String error){
        try{
            DriverStation.reportError(error, false);
        }catch(Exception e){
            DriverStation.reportWarning(e.toString(), false);
        }
    }

    public void error(Double error){
        try{
            DriverStation.reportError(error.toString(), false);
        }catch(Exception e){
            DriverStation.reportWarning(e.toString(), false);
        }
    }

    public void error(int error){
        try{
            String errorString = Integer.toString(error);
            DriverStation.reportError(errorString, false);
        }catch(Exception e){
            DriverStation.reportWarning(e.toString(), false);
        }
    }

    public void error(boolean error){
        try{
            String errorString = Boolean.toString(error);
            DriverStation.reportError(errorString, false);
        }catch(Exception e){
            DriverStation.reportWarning(e.toString(), false);
        }
    }

    public void metric(String name, String value) {
        try{
            SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
        }catch(Exception e){
            DriverStation.reportWarning(e.toString(), true);
        }
    }

    public void metric(String name, double value) {
        try{
            SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
        }catch(Exception e){
            DriverStation.reportWarning(e.toString(), true);
        }
    }

    public void metric(String name, boolean value) {
        try{
            SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
        }catch(Exception e){
            DriverStation.reportWarning(e.toString(), true);
        }
    }
}