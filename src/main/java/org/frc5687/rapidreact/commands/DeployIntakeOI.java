package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Intake;

/** Deploy intake to pick up balls */
public class DeployIntakeOI extends OutliersCommand {

    private final Intake _intake;

    /** Create DeployIntake command for teleop mode
     * 
     * @param intake passed from RobotContainer
     */
    public DeployIntakeOI(Intake intake) {
        _intake = intake;
        addRequirements(_intake);
    }
   
    @Override
    public void initialize() {
        super.initialize();
        _intake.deploy();
    }

    @Override
    public void execute() {
        super.execute();
        // spin
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
