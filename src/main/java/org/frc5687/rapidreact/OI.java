/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * Has some instructions on how to switch controls.
 * See ButtonMap for configuration of joystick and gamepad.
*/
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;

import org.frc5687.rapidreact.commands.SnapTo;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.util.AxisButton;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;

/**
 * To add a button to control a subsystem there are a number of steps needed.  I'll use SHOOT as an example:
 * 1) Define the button in the ButtonMap.Buttons class (in ButtonMap.java):
 * 
 *         public static class SHOOT {
 *           public static int Controller = Controllers.DRIVER_JOYSTICK;
 *           public static int Button = 0;
 *         }
 *
 * 2) Add a private member variable for the button:
 * 
 *        private JoystickButton _shootButton;
 * 
 * 3) Instantiate the button in the OI() constructor, referencing the new ButtonMap entry:
 * 
 *        _shootButton = addJoystickButton(ButtonMap.Buttons.SHOOT.Controller, ButtonMap.Buttons.SHOOT.Button);
 * 
 * 4) Add the subsystem to the signature for OI.initializeButtons:
 * 
 *        public void initializeButtons(DriveTrain driveTrain, Shooter shooter)
 * 
 * 5) Initialize the button in OI.initializeButtons:
 * 
 *        _shootButton.whenHeld(new Shoot(shooter));
 * 
 * 6) Add the subsystem to the RobotContainer.init() call to _oi.inializeButtons:
 * 
 *        _oi.initializeButtons(_driveTrain, _shooter);
 */

 /**
 * OI is the operator input class
 * 
 * <p>Define the types of devices that can provide operator input
 * (joysticks, gamepads, drive wheels, keyboards, etc.)
 */
public class OI extends OutliersProxy {

    private Rotation2d theta;
    // "Raw" joystick values
    private double yIn = 0;
    private double xIn = 0;

    private static final int MAX_USB_PORTS = 10;

    private Joystick[] _joysticks = new Joystick[MAX_USB_PORTS];

    // private JoystickButton _shootButton;
    private JoystickButton _resetNavX;
    private JoystickButton _snapBTN;

    public OI() {
        addJoystick(ButtonMap.Controllers.TRANSLATOR_JOYSTICK);
        addJoystick(ButtonMap.Controllers.ROTATOR_JOYSTICK);
        addGamepad(ButtonMap.Controllers.ROTATOR_GAMEPAD);

        // _shootButton = addJoystickButton(ButtonMap.Buttons.SHOOT.Controller, ButtonMap.Buttons.SHOOT.Button);
        _resetNavX = addJoystickButton(ButtonMap.Buttons.RESET_NAVX.Controller, ButtonMap.Buttons.RESET_NAVX.Button);
    }

    public void initializeButtons(DriveTrain driveTrain/*, Shooter shooter*/) {
        // _resetNavX.whenReleased(driveTrain::resetNavX);   

        // example of creating shoot button.
        // _shootButton.whenHeld(new Shoot(shooter));
        _resetNavX = new JoystickButton(_joysticks[0], 5);
        _snapBTN = new JoystickButton(_joysticks[1], 4);
        _snapBTN.whenHeld(new SnapTo(driveTrain, theta));
    }


    public double getDriveY() {
        Joystick translation = getJoystick(ButtonMap.Axes.Translation.Controller);

        yIn = getSpeedFromAxis(translation, ButtonMap.Axes.Translation.Y);
        yIn = applyDeadband(yIn, Constants.DriveTrain.DEADBAND);

        // TODO: explain the following magic
        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        Joystick translation = getJoystick(ButtonMap.Axes.Translation.Controller);

        // TODO: explain the negative sign here
        xIn = -getSpeedFromAxis(translation, ButtonMap.Axes.Translation.X);
        xIn = applyDeadband(xIn, Constants.DriveTrain.DEADBAND);

        // TODO: explain the following magic
        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        xOut = (xOut + (xIn * 2)) / 3.0;
        return xOut;
    }

    public double getRotationX() {
        Joystick rotation = getJoystick(ButtonMap.Axes.Rotation.Controller);

        double speed = getSpeedFromAxis(rotation, ButtonMap.Axes.Rotation.Twist);
        speed = applyDeadband(speed, 0.2);

        return speed;
    }

    protected double getSpeedFromAxis(Joystick joystick, int axisNumber) {
        return joystick.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {}

    /**
     * Instantiates a Joystick on the specified port and adds it to the _joysticks array.
     * 
     * @param port Pass -1 to skip this joystick.
     */
    private void addJoystick(int port) {
        if (port < 0) { return; }
        _joysticks[port] = new Joystick(port);
    }

    /**
     * Instantiates Gamepad on the specified port and adds it to the _joysticks array.
     * 
     * @param port Pass -1 to skip this gamepad.
     */
    private void addGamepad(int port) {
        if (port < 0) { return; }
        _joysticks[port] = new Gamepad(port);
    }

    /**
     * Instantiates Button on the specific controller and adds it to the _buttons array.
     * first "number of buttons" is for the first controller, seconds "number of buttons" is for the 2nd controller, etc.
     * @param button
     */
    private JoystickButton addJoystickButton(int controller, int buttonNumber) {
        Joystick joystick = getJoystick(controller);
        return new JoystickButton(joystick, buttonNumber);
    }
    /**
     * Returns the joystick assigned to a specific port.  Returns null is no joystick assigned or port is -1.
     * @param port
     * @return
     */
    private Joystick getJoystick(int port) {
        if (port < 0) { return null; }
        return _joysticks[port];
    }

    private AxisButton addAxisButton(int port, int buttonNumber, double threshold) {
        Joystick joystick = getJoystick(port);
        return new AxisButton(joystick, buttonNumber, threshold);
    }

}
