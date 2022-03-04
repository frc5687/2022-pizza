package org.frc5687.rapidreact;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.AddressableLED;

public class Lights {

    private AddressableLEDBuffer _blinkin;
    private Spark _blinkens;
    private AddressableLED _blinkinAddress;
    
    public Lights(int blinkenPort, int length) {
        _blinkens = new Spark(1);
    }

    public void stop(){
        //Stop blinkins
        _blinkinAddress.stop();
    }

    public void setRed() {
        _blinkens.set(0.87);
    }
}
