package org.frc5687.rapidreact.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.rapidreact.RobotContainer;

/**
 * Shoot one ball then taxi out of tarmac.
 */
public class AutoOneBall extends SequentialCommandGroup {
    
    /**
     * Create an AutoOneBall sequential command group
     * 
     * @param robotContainer
     */
    public AutoOneBall(RobotContainer robot) {

        addCommands(
            new DeployIntake(robot.intake),
            new Aim(robot.driveTrain),
            new Shoot(robot.catapult),
            new AutoZeroBall(robot)
        );
    }

}
