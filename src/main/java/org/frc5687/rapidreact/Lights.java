package org.frc5687.rapidreact;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED;

public class Lights {

    private AddressableLEDBuffer _blinkin;
    private AddressableLED _blinkinAddress;
    
    public Lights(int blinkenPort, int length) {
        //Set the port number
        _blinkinAddress = new AddressableLED(blinkenPort);
        //Configure length
        _blinkin = new AddressableLEDBuffer(length);
        _blinkinAddress.setLength(_blinkin.getLength());
        //Start blinkins
        _blinkinAddress.setData(_blinkin);
        _blinkinAddress.start();
    }

    public void stop(){
        //Stop blinkins
        _blinkinAddress.stop();
    }

    public void setRed() {
        for (var i = 0; i < _blinkin.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            _blinkin.setRGB(i, 255, 0, 0);
         }
         _blinkinAddress.setData(_blinkin);
    }
}
