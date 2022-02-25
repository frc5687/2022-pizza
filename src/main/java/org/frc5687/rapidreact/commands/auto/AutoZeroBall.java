package org.frc5687.rapidreact.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.rapidreact.RobotContainer;

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

        // Destination A
        double xPos = -1.0;
        double yPos = -1.0;
        double theta = 0.0;
        double omega = -0.5;
        double velocity = 1.0;
        DriveToPose driveToA = new DriveToPose(
            robot.driveTrain, xPos, yPos, theta, omega, velocity);
        
        addCommands(
            driveToA
        );
    }

}
