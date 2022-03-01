package org.frc5687.rapidreact.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import org.frc5687.rapidreact.RobotContainer;

import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Intake;

/**
 * Shoot one ball then taxi out of tarmac.
 */
public class AutoOneBall extends SequentialCommandGroup {
    
    /**
     * Create an AutoOneBall sequential command group
     * 
     * @param robotContainer
     */
    public AutoOneBall(
        Catapult catapult,
        DriveTrain driveTrain,
        Indexer indexer,
        Intake intake
        ) {

        addCommands(
            new DeployIntake(intake),
            new Aim(driveTrain),
            new Shoot(catapult),
            new AutoZeroBall(
                catapult,
                driveTrain,
                indexer,
                intake)
        );
    }

}
