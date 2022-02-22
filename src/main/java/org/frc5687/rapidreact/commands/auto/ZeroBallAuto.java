package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.commands.DriveAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * ZeroBall auto will drive robot out of tarmac.
 */
public class ZeroBallAuto extends SequentialCommandGroup {

    private final Pose2d _destination;
    // private final Rotation2d _theta;

    public ZeroBallAuto(
        DriveTrain driveTrain,
        Pose2d destination
    ) {
        // Was hardcoded for testing
        // _theta = new Rotation2d(0.0);
        //_destination = new Pose2d(1.0, 1.0, _theta);

        // Now get destination from caller
        _destination = destination;

        addCommands(
            new DriveAuto(driveTrain, _destination)
         );
    }

}
