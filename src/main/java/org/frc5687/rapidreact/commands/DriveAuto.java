/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands;


import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.filter.SlewRateLimiter;

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

    public DriveAuto(DriveTrain driveTrain, Pose2d pose) {
        _driveTrain = driveTrain;
        _destination = pose;
        // _vxFilter = new SlewRateLimiter(3.0);
        // _vyFilter = new SlewRateLimiter(3.0);
        addRequirements(_driveTrain);
    }

    @Override
    public void initialize() {
        _heading = new Rotation2d(0.0);
        _velocity = 0.1;

        super.initialize();
        _driveTrain.startModules();
    }

    @Override
    public void execute() {
        super.execute();

        //  driveX and driveY are swapped due to coordinate system that WPILib uses.
        // TODO: verify this description of vx and vy is accurate

        /**
         * Based on observation, appears that
         * 
         *             North = -Y
         *  West = +X              East = -X
         *             South = +Y
         * 
         */

        // Double vx = _vxFilter.calculate(0.0) * Constants.DriveTrain.MAX_MPS;
        // Double vy = _vyFilter.calculate(0.0) * Constants.DriveTrain.MAX_MPS;
        // Double rot = 0.0;

        // _driveTrain.drive(vx, vy, rot, true);

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
