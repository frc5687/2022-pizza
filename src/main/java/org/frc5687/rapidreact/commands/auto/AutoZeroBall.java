package org.frc5687.rapidreact.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.rapidreact.RobotContainer;
import org.frc5687.rapidreact.Constants;

/**
 * Taxi out of tarmac.
 */
public class AutoZeroBall extends SequentialCommandGroup {
    
    /**
     * Create an AutoZeroBall sequential command group
     * 
     * @param robotContainer
     */
    public AutoZeroBall(RobotContainer robot) {

        double xPos;
        double yPos;
        double theta;
        double omega;
        double velocity;

        // Choose destination depending on starting position
        switch(Constants.Auto.AUTO_POSITION) {
            case OBT_LEFT:
            case OBT_RIGHT:
            case TBT_LEFT:
            case TBT_RIGHT:
            default:
                xPos = -1.0;
                yPos = -1.0;
                theta = 0.0;
                omega = -0.5;
                velocity = 1.0;    
        }
        
        addCommands(
            new DriveToPose(robot.driveTrain, xPos, yPos, theta, omega, velocity)
        );
    }

}
