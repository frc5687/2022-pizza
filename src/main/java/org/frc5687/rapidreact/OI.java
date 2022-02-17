/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * Has some instructions on how to switch controls.
*/
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;

import org.frc5687.rapidreact.Constants.Lime;
import org.frc5687.rapidreact.commands.Aim;
import org.frc5687.rapidreact.commands.SnapTo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.Limelight;
import org.frc5687.rapidreact.util.OutliersProxy;

/** 
 * OI is the operator input class
 * 
 * <p>Define the types of devices that can provide operator input
 * (joysticks, gamepads, drive wheels, keyboards, etc.)
 */
public class OI extends OutliersProxy {
    // Joysticks and XBox controller
    protected Gamepad _gamepad;
    protected Joystick _rotation;
    protected Joystick _translation;

    private Rotation2d theta;
    private JoystickButton _resetNavX;
    private JoystickButton _snapBTN;
    private JoystickButton _aim;
    // "Raw" joystick values
    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _gamepad = new Gamepad(0);
        _translation = new Joystick(0);
        _rotation = new Joystick(1);
        _resetNavX = new JoystickButton(_translation, 5);
        _snapBTN = new JoystickButton(_gamepad, Gamepad.Buttons.A.getNumber());
        _aim = new JoystickButton(_gamepad, Gamepad.Buttons.X.getNumber());
    }

    public void initializeButtons(DriveTrain driveTrain, Limelight limelight) {
        _aim.whenHeld(new Aim(driveTrain, limelight));
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
        theta = new Rotation2d(speed);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {}
}
