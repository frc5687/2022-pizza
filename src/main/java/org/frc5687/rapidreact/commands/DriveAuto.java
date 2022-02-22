/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.subsystems.DriveTrain;

/**
 * Drive in autonomous mode (i.e., no OI control)
 */
public class DriveAuto extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final SlewRateLimiter _vxFilter;
    private final SlewRateLimiter _vyFilter;

    public DriveAuto(DriveTrain driveTrain) {
        _driveTrain = driveTrain;
        _vxFilter = new SlewRateLimiter(3.0);
        _vyFilter = new SlewRateLimiter(3.0);
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
        // TODO: verify this description of vx and vy is accurate

        /**
         * Based on observation, appears that
         * 
         *             North = -Y
         *  West = +X              East = -X
         *             South = +Y
         * 
         */

        Double vx = _vxFilter.calculate(0.0) * Constants.DriveTrain.MAX_MPS;
        Double vy = _vyFilter.calculate(0.0) * Constants.DriveTrain.MAX_MPS;
        Double rot = 0.0;

        _driveTrain.drive(vx, vy, rot, true);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }


    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
