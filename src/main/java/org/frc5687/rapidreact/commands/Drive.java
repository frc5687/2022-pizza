package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.subsystems.DriveTrain;

import edu.wpi.first.math.filter.SlewRateLimiter;

import org.frc5687.rapidreact.config.Constants;
import org.frc5687.rapidreact.config.Constants.DriveTrain.*;

public class Drive extends OutliersCommand {

    private final DriveTrain _driveTrain;
    private final SlewRateLimiter _vxFilter;
    private final SlewRateLimiter _vyFilter;

    private final OI _oi;

    public Drive(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _vxFilter = new SlewRateLimiter(6.0);
        _vyFilter = new SlewRateLimiter(6.0);
        addRequirements(_driveTrain);
//        logMetrics("vx","vy");
//        enableMetrics();
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
        double vx = _vxFilter.calculate(-_oi.getDriveY()) * (Constants.DriveTrain.MAX_MPS);
        double vy = _vyFilter.calculate(_oi.getDriveX()) * (Constants.DriveTrain.MAX_MPS);
        metric("Robot heading", _driveTrain.getHeading().getRadians());
        double rot = _oi.getRotationX() * Constants.DriveTrain.MAX_ANG_VEL;
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