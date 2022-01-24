/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * Has some instructions on how to switch controls.
*/
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;

import org.frc5687.rapidreact.commands.DriveTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;

public class OI extends OutliersProxy {
    protected Gamepad _gamepad;
    protected Joystick _rotation;
    protected Joystick _translation;

    private JoystickButton _autoButton;
    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        // _gamepad = new Gamepad(0);

        _translation = new Joystick(0);
        _rotation = new Joystick(1);
        
        _autoButton = new JoystickButton(_translation, 5);
    }

    public void initializeButtons(DriveTrain driveTrain, Trajectory trajectory) {
        _autoButton.whenPressed(new DriveTrajectory(driveTrain, trajectory));
    }

    public double getDriveY() {
        //Comment for gamepad control
        yIn = getSpeedFromAxis(_translation, _translation.getYChannel());
        //Uncomment for gamepad control
        // yIn = getSpeedFromAxis(Gamepad, Gamepad.getYChannel());
        yIn = applyDeadband(yIn, Constants.DriveTrain.DEADBAND);

        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        //Comment for gamepad control
        xIn = -getSpeedFromAxis(_translation, _translation.getXChannel());
        //Uncomment for gamepad control
        //xIn = -getSpeedFromAxis(Gamepad, Gamepad.getXChannel());
        xIn = applyDeadband(xIn, Constants.DriveTrain.DEADBAND);
        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        xOut = (xOut + (xIn * 2)) / 3.0;
        return xOut;
    }

    public double getRotationX() {
        double speed = getSpeedFromAxis(_rotation, _rotation.getXChannel());
        speed = applyDeadband(speed, 0.2);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {}
}
