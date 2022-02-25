package org.frc5687.rapidreact.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.rapidreact.RobotContainer;
import org.frc5687.rapidreact.config.Constants;

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
        // For now, this is set in Constants
        // TODO: change to get position from AutoChooser
        switch(Constants.Auto.AUTO_POSITION) {
            case TBT_RIGHT:
                xPos = 0.5;
                yPos = -1.0;
                theta = 0.0;
                omega = 0.0;
                velocity = 1.0;
                break;
            case TBT_LEFT:
                xPos = -1.5;
                yPos = 0.5;
                theta = 0.0;
                omega = 0.0;
                velocity = 1.0;
                break;
            case OBT_RIGHT:
                xPos = -1.0;
                yPos = 0.0;
                theta = 0.0;
                omega = 0.0;
                velocity = 1.0;
                break;
            case OBT_LEFT:
                xPos = -1.0;
                yPos = 1.0;
                theta = 0.0;
                omega = 0.0;
                velocity = 1.0;
                break;
            default:
                // assume OBT_RIGHT
                xPos = -1.0;
                yPos = 0.0;
                theta = 0.0;
                omega = 0.0;
                velocity = 1.0;    
        }
        
        addCommands(
            new DriveToPose(robot.driveTrain, xPos, yPos, theta, omega, velocity)
        );
    }

}
