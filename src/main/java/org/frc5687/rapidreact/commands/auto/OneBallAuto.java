package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.commands.auto.DriveToPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneBallAuto extends SequentialCommandGroup{
    public OneBallAuto (
        DriveTrain driveTrain,
        //Catapult catapult,
        Pose2d destination
    ) {
        addCommands(
            //shoot
            new DriveToPose(driveTrain, destination, destination.getRotation(), 0.2)
        );
    }
}
