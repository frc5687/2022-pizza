/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * Has some instructions on how to switch controls.
 * See ButtonMap for configuration of joystick and gamepad.
*/
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;

import org.frc5687.rapidreact.Constants.Lime;
import org.frc5687.rapidreact.commands.Aim;
import org.frc5687.rapidreact.commands.SnapTo;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Intake;

// import org.frc5687.rapidreact.commands.ResetNavX;

// import org.frc5687.rapidreact.util.AxisButton;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.Limelight;
import org.frc5687.rapidreact.util.OutliersProxy;
import static org.frc5687.rapidreact.util.Helpers.*;

/**
 * To add a button to control a subsystem there are a number of steps needed.  I'll use SHOOT as an example:
 * 
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
    private JoystickButton _resetNavX;
    private JoystickButton _snapBTN;
    private JoystickButton _aim;
    // "Raw" joystick values
    private double yIn = 0;
    private double xIn = 0;

    private static final int MAX_USB_PORTS = 10;

    private Joystick[] _joysticks = new Joystick[MAX_USB_PORTS];

    // Allocate buttons
    private JoystickButton _resetNavX;
    private JoystickButton _snapBTN;

    public OI() {
        addJoystick(ButtonMap.Controllers.TRANSLATOR_JOYSTICK);
        addJoystick(ButtonMap.Controllers.ROTATOR_JOYSTICK);
        addGamepad(ButtonMap.Controllers.ROTATOR_GAMEPAD);

        // Create buttons
        _resetNavX = addJoystickButton(ButtonMap.Buttons.RESET_NAVX.Controller, ButtonMap.Buttons.RESET_NAVX.Button);
        _snapBTN = new JoystickButton(_joysticks[1], 4);

    }

    /** Define how buttons work:
     *  - when button calls command (when pressed, released, held, etc.)
     *  - which command gets called
     */
    public void initializeButtons(
        DriveTrain driveTrain,
        Indexer indexer,
        Intake intake
        ) {
        _resetNavX.whenPressed(new InstantCommand(driveTrain::resetYaw, driveTrain));
        // _resetNavX.whenReleased(new ResetNavX(driveTrain));
        // _snapBTN.whenHeld(new SnapTo(driveTrain, theta));
    }

    public double getDriveY() {
        Joystick translation = getJoystick(ButtonMap.Axes.Translation.Controller);

        yIn = getSpeedFromAxis(translation, ButtonMap.Axes.Translation.Y);
        yIn = applyDeadband(yIn, Constants.DriveTrain.DEADBAND);

        return circularize(yIn, xIn);
    }

    public double getDriveX() {
        Joystick translation = getJoystick(ButtonMap.Axes.Translation.Controller);

        // TODO: explain the negative sign here
        xIn = -getSpeedFromAxis(translation, ButtonMap.Axes.Translation.X);
        xIn = applyDeadband(xIn, Constants.DriveTrain.DEADBAND);

        return circularize(xIn, yIn);
    }

    /**
     * Change joystick output in corners to approximate movement around a circle
     * I.e., have a constant velocity when moving in a straight line or diagonally
     * 
     * TODO: use some real math to circularize joystick input
     * 
     * @param a xIn or yIn
     * @param b yIn or xIn
     * @return xIn or yIn circularized
     */
    private double circularize(double a, double b) {

        /**
         * Problem is joystick moves in a square box but we want to emulate
         * a circle so velocity stays constant whether you are moving
         * along an axis (with 0 X or Y velocity component) or along a
         * diagonal (with both X and Y velocity components).
         * 
         * For example, if joystick is at NE corner, X and Y velocity inputs
         * are both 1, so total velocity is sqrt of 2 (Pythagorean theorem).
         * But if joystick is at E side, velocity is only 1.
         * 
         * As joystick moves along edge of box, we want to recalculate both
         * X and Y input to emulate moving along arc of circle.  At NE corner,
         * X and Y input should both be sqrt of 0.5.  That would make total
         * velocity 1.
         */

        double c = a / (Math.sqrt(a * a + (b * b)) + Constants.EPSILON);
        c = (c + (a * 2)) / 3.0;
        return c;
    }

    public double getRotationX() {
        Joystick rotation = getJoystick(ButtonMap.Axes.Rotation.Controller);

        double speed = getSpeedFromAxis(rotation, ButtonMap.Axes.Rotation.Twist);
        speed = applyDeadband(speed, 0.2);
        theta = new Rotation2d(speed);
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

    /*
    private AxisButton addAxisButton(int port, int buttonNumber, double threshold) {
        Joystick joystick = getJoystick(port);
        return new AxisButton(joystick, buttonNumber, threshold);
    }
    */

}
