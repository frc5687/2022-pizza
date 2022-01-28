package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.commands.DriveTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class ZeroBallAuto extends SequentialCommandGroup{
    public ZeroBallAuto(
        DriveTrain driveTrain,
        Trajectory trajectory
    ) {
        addCommands(
            new DriveTrajectory(driveTrain, trajectory)
         );
    }
}
