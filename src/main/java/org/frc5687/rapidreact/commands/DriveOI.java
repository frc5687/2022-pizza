/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import org.frc5687.rapidreact.Constants;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.OI;

/**
 * Drive using OI control
 * 
 * <p>This is the default command for DriveTrain, so requires subsystem and
 * never finishes.
 */
public class DriveOI extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final SlewRateLimiter _vxFilter;
    private final SlewRateLimiter _vyFilter;

    private final OI _oi;

    public DriveOI(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _vxFilter = new SlewRateLimiter(3.0);
        _vyFilter = new SlewRateLimiter(3.0);
        addRequirements(_driveTrain); // necessary to be DriveTrain's default command
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
        double vx = _vxFilter.calculate(-_oi.getDriveY()) * Constants.DriveTrain.MAX_MPS;
        double vy = _vyFilter.calculate(_oi.getDriveX()) * Constants.DriveTrain.MAX_MPS;
        double rot = -_oi.getRotationX() * Constants.DriveTrain.MAX_ANG_VEL;
        _driveTrain.drive(vx, vy, rot, true);
    }

    @Override
    public boolean isFinished() {
        return false; // as a default command, never finish
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
