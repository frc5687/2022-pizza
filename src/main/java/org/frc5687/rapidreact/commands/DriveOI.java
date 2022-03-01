/* Team 5687 (C)2021-2022 */
package org.frc5687.rapidreact.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;

import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.OI;
import org.frc5687.rapidreact.config.Constants;

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

    /**
     * Create DriveOI command
     * 
     * <p>Default command for DriveTrain.  Drive under OI control.
     * 
     * @param driveTrain
     * @param oi
     */
    public DriveOI(DriveTrain driveTrain, OI oi) {
        _driveTrain = driveTrain;
        _oi = oi;
        _vxFilter = new SlewRateLimiter(Constants.DriveTrain.SLEW_LIMIT_X);
        _vyFilter = new SlewRateLimiter(Constants.DriveTrain.SLEW_LIMIT_Y);
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

        // Set vx, vy and vtheta based on joystick input
        double vx = _vxFilter.calculate(_oi.getDriveX()) * Constants.DriveTrain.MAX_MPS;
        double vy = _vyFilter.calculate(_oi.getDriveY()) * Constants.DriveTrain.MAX_MPS;

        // TODO: add snap-to here to set angular velocity of robot

        /**
         * Here's the plan:
         * 1. Use a gamepad for rotation so we have two joysticks.
         * 2. Left joystick is for manual rotation (left and right robot-reference rotation)
         * 3. Right joystick for "snap-to" auto rotation (direction is field-reference rotation)
         * 4. Snap-to overrides manual rotation, i.e.
         *    a. If right joystick is out of deadband, use its heading to control heading of robot
         *    b. Else, use left joystick for manual rotation
         */

        // convert clockwise to counter-clockwise for rotation
        double vomega = _oi.getRotation() * Constants.DriveTrain.MAX_ANG_VEL; // manual rotation

        _driveTrain.drive(vx, vy, vomega, true);
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
