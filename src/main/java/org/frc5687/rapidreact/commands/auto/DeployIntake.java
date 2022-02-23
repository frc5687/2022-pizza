package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Intake;

/**
 * Deploy intake so we can shoot catapult
 */
public class DeployIntake extends OutliersCommand {

    private final Intake _intake;

    /** constructor */
    public DeployIntake(Intake intake) {
        _intake = intake;
    }
   
    @Override
    public void initialize() {
        super.initialize();
        _intake.deploy();
    }

    @Override
    public void execute() {
        super.execute();
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return _intake.isDeployed();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
 
}
