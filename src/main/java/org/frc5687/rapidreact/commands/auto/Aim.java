/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;

/** Aim in autonomous mode (i.e., no OI control) to shoot ball. */
public class Aim extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private Rotation2d _heading;

    /** Create auto Aim command
     * 
     * @param driveTrain pass in from RobotContainer
     */
    public Aim(
        DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        // Calculate heading based on relative position of robot and hub
        _heading = new Rotation2d(0.0);
        _driveTrain.startModules();
    }

    @Override
    public void execute() {
        super.execute();

        // Aim robot by moving drive train
    }

    @Override
    public boolean isFinished() {
        return _driveTrain.isAtRotation(_heading);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
