package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.util.OutliersContainer;

//import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Lights extends OutliersSubsystem{

    private Spark _blinkens;
    //private DriverStation.Alliance _alliance;
    private boolean _color = false;

    
    public Lights(OutliersContainer container, int blinkenPort, int length) {
        super(container);
        _blinkens = new Spark(1);
    }

    public void initialize() {
        //_alliance = DriverStation.getAlliance();
    }

    public void stop(){
        //Stop blinkins
        
    }

    public void setRed() {
        _blinkens.set(0.87);
        _color = true;
    }

    @Override
    public void updateDashboard() {
        metric("Running", _color);
    }
}