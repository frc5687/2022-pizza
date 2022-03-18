/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.Catapult;

/** Shoot ball in autonomous mode (i.e., no OI control). */
public class Shoot extends OutliersCommand {

    private final Catapult _catapult;

    /**
     * Create auto Shoot command
     * 
     * @param catapult pass in from RobotContainer
     */
    public Shoot(Catapult catapult) {
        _catapult = catapult;
        addRequirements(_catapult);
    }

    @Override
    public void initialize() {
        super.initialize();
        // Shoot catapult
    }

    @Override
    public void execute() {
        super.execute();
        // Reset catapult
    }

    @Override
    public boolean isFinished() {
        return _catapult.isReset();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
