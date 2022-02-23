/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands;


import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.filter.SlewRateLimiter;

/**
 * Drive in autonomous mode (i.e., no OI control)
 */
public class DriveAuto extends OutliersCommand {

    private final Pose2d _destination;
    private final DriveTrain _driveTrain;
    // private final SlewRateLimiter _vxFilter;
    // private final SlewRateLimiter _vyFilter;

    private Rotation2d _heading;
    private Double _velocity;

    public DriveAuto(
        DriveTrain driveTrain,
        Pose2d pose,
        Rotation2d heading,
        Double velocity) {
        _driveTrain = driveTrain;
        _destination = pose;
        _heading = heading;
        _velocity = velocity;
        // _vxFilter = new SlewRateLimiter(3.0);
        // _vyFilter = new SlewRateLimiter(3.0);
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

        //  driveX and driveY are swapped due to coordinate system that WPILib uses.

        /**
         * Based on observation, appears that
         * 
         *             North = +Y
         *  West = +X              East = -X
         *             South = -Y
         * 
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
