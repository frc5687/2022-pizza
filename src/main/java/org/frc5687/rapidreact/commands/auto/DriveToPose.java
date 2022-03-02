/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.commands.OutliersCommand;
import org.frc5687.rapidreact.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Drive in autonomous mode (i.e., no OI control) to a field-relative pose. */
public class DriveToPose extends OutliersCommand {

    private final DriveTrain _driveTrain;

    private final Pose2d _destination;
    private Rotation2d _heading;
    private Double _velocity;

    /** Create DriveToPose command
     * 
     * @param driveTrain pass in from RobotContainer
     * @param xPos meters
     * @param yPos meters
     * @param theta fraction of radians
     * @param omega fraction of radians
     * @param velocity m/s
     */
    public DriveToPose(
        DriveTrain driveTrain,
        double xPos,
        double yPos,
        double theta,
        double omega,
        double velocity) {

        _driveTrain = driveTrain;
        double _theta = theta * Math.PI;
        double _omega = omega * Math.PI;
        _destination = new Pose2d(xPos, yPos, new Rotation2d(_theta));
        _heading = new Rotation2d(_omega);
        _velocity = velocity;
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.startModules();
    }

    @Override
    public void execute() {
        super.execute();

        /**
         * According to HolonomicDriveController:
         * 
         * Positive X is away from your alliance wall.
         * Positive Y is to your left when standing behind your alliance wall.
         * Robot's angle is considered 0 when it is facing directly away
         * from your alliance wall.
         * Turning left (CCW) is positive.
         * 
         *             North = +X
         *  West = +Y              East = -Y
         *             South = -X
         */

        _driveTrain.poseFollower(_destination, _heading, _velocity);
    }

    @Override
    public boolean isFinished() {
        return _driveTrain.isAtPose(_destination);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
