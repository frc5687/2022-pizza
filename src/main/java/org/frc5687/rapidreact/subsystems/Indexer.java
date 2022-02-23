package org.frc5687.rapidreact.subsystems;

import edu.wpi.first.wpilibj.Servo;

/**
 * Keep balls separated so can intake two, shoot just one.
 * 
 * <p>Note: 2022-robot repo calls this subsystem "ServoStop"
 */
public class Indexer {
    
    private Servo stopper;
    private boolean feeding = false;

    public Indexer() {
        stopper = new Servo(9);
    }

    public void lower() {
        //Lowers the blocking arm
        //Lets balls enter the catapult
        feeding = false;
        stopper.setAngle(180);
    }

    public void raise() {
        //Raises the blocking arm
        //Stops balls from enter the catapult
        feeding = true;
        stopper.setAngle(60);
    }

    public boolean getMode() {
        //Check if the indexer arm is feeding the catapult
        return feeding;
    }
}
