package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.commands.DriveTrajectory;
import org.frc5687.rapidreact.commands.TotallyLegitShoot;   
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneBallAuto extends SequentialCommandGroup{
    public OneBallAuto (
        DriveTrain driveTrain,
        Trajectory trajectory
        
    ) {
        addCommands(
            new ParallelDeadlineGroup(
                new DriveTrajectory(driveTrain, trajectory),
                new TotallyLegitShoot(driveTrain)
            )
         );
    }
}
