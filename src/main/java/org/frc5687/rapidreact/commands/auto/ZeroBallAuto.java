package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.commands.DriveAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * TODO: Explain ZeroBallAuto command
 */
public class ZeroBallAuto extends SequentialCommandGroup {

    public ZeroBallAuto(
        DriveTrain driveTrain
    ) {
        addCommands(
            new DriveAuto(driveTrain)
         );
    }

}
