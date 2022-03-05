/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * @see JoystickMap for configuration of joystick and gamepad.
*/
package org.frc5687.rapidreact;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.frc5687.rapidreact.config.JoystickMap;
import org.frc5687.rapidreact.config.Constants;

import org.frc5687.rapidreact.commands.DeployIntakeOI;

import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;
import static org.frc5687.rapidreact.util.Helpers.*;

 /** Operator input
 * 
 * <p> Define the types of devices that can provide operator input
 * (joysticks, gamepads, drive wheels, keyboards, etc.)
 * 
 * @see JoystickMap
 */
public class OI extends OutliersProxy {

    // private RobotContainer _robot;

    // Declare joysticks
    private Joystick _translation;
    private Joystick _rotation;
    private Joystick _debug;

    // Declare joystick axes and signs
    private int _xAxis; // which axis controls forward / back
    private int _xSign; // whether to invert joystick value
    private int _yAxis; // which axis controls left / right
    private int _ySign;
    private int _twistAxis; // which axis controls rotation
    private int _twistSign;

    /** Commands that can be mapped to buttons */
    public static enum Command {
        NOT_IN_USE,
        AUTO_RUN,
        CATAPULT_SHOOT,
        INTAKE_DEPLOY,
        NAVX_RESET
    }    

    // Declare joystick buttons
    private JoystickButton _autoRun;
    private JoystickButton _catapultShoot;
    private JoystickButton _intakeDeploy;
    private JoystickButton _navXReset;

    // Button mappings
    private Command[] _translationButtons;
    private Command[] _rotationButtons;
    private Command[] _debugButtons;

    // "Raw" joystick values
    private double yIn = 0;
    private double xIn = 0;

    /** Create an OI */
    public OI() {

        createJoysticks();
        createButtons();

    }

    /** Assign a new JoystickButton for a command.
     * 
     * @param joystick
     * @param buttonNumber
     * @param command
     */
    private void addJoystickButton(Joystick joystick, int buttonNumber, Command command) {
        switch(command) {
            case NOT_IN_USE:
                return;
            case AUTO_RUN:
                _autoRun = new JoystickButton(joystick, buttonNumber);
                break;
            case CATAPULT_SHOOT:
                _catapultShoot = new JoystickButton(joystick, buttonNumber);
                break;
            case INTAKE_DEPLOY:
                _intakeDeploy = new JoystickButton(joystick, buttonNumber);
                break;
            case NAVX_RESET:
                _navXReset = new JoystickButton(joystick, buttonNumber);
                break;
        }
    }

    /** Define how buttons work
     * 
     * <ul>
     *  <li> when button calls command (when pressed, released, held, etc.)
     *  <li> which command gets called
     * </ul>
     * 
     * @param robot the RobotContainer initializing buttons
     */
    public void initializeButtons(RobotContainer robot) {
        // We may or may not have assigned each command to a button.
        // So check for null before initializing button method.
        // if (_autoRun != null) {
        //     _autoRun.whenPressed(new AutoOneBall(robot));
        // }
        if (_intakeDeploy != null) {
            _intakeDeploy.whenHeld(new DeployIntakeOI(robot.intake));
        }
        if (_navXReset != null) {
            _navXReset.whenReleased(
                new InstantCommand(robot.driveTrain::resetYaw, robot.driveTrain));
        }
        if (_catapultShoot != null) {
            _catapultShoot.whenPressed(
                new InstantCommand(robot.catapult::shoot, robot.catapult));
        }
    }

    // Get movement values from joysticks

    // Goal is to move the robot relative to field reference.

    // According to WPI's kinematics classes:
    // Positive x is away from your alliance wall.
    // Positive y is to your left when standing behind the alliance wall.
    // Positve theta (rotation) is counter clockwise (CCW).

    /** Get X value from translation joystick
     * 
     * <p> Robot forward and backward motion (away or toward). Forward +x.
     */
    public double getDriveX() {
   
        xIn = _xSign * getSpeedFromAxis(_translation, _xAxis);
        xIn = applyDeadband(xIn, Constants.DriveTrain.DEADBAND_TRANSLATION);
        return circularize(xIn, yIn);
    }

    /** Get Y value from translation joystick 
     * 
     * <p> Robot sideways motion (left or right). Left +y.
    */
    public double getDriveY() {
        yIn = _ySign * getSpeedFromAxis(_translation, _yAxis);
        yIn = applyDeadband(yIn, Constants.DriveTrain.DEADBAND_TRANSLATION);
        return circularize(yIn, xIn);
    }

    /** Get rotation value from rotation joystick
     * 
     * <p> Left +omega.
    */
    public double getRotation() {
        double rotIn = _twistSign * getSpeedFromAxis(_rotation, _twistAxis);
        rotIn = applyDeadband(rotIn, Constants.DriveTrain.DEADBAND_ROTATION);
        return rotIn;
    }

    // Helper methods

    /** Read joystick value */
    protected double getSpeedFromAxis(Joystick joystick, int axisNumber) {
        return joystick.getRawAxis(axisNumber);
    }

    /**
     * Change joystick output in corners to emulate movement around a circle.
     * I.e., have a constant velocity when moving in a straight line or diagonally
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
         * X and Y input to emulate moving along arc of circle.
         * 
         * Set Z to be radius of circle.
         * 
         * cos(omega) = X / Z
         * X = cos(omega) * Z
         * 
         * Since our max velocity is 1, we can set Z = 1.  So for any angle
         * that our joystick is at, the maximum X is cos(omega) to keep our
         * velocity at or below 1.
         * 
         */

        // Find angle of joystick position
        double angle = new Rotation2d(a, b).getRadians();

        // Max velocity is 1.  So we can use cos(omega) as our max
        // velocity in this direction.
        if (Math.abs(a) > Math.abs(Math.cos(angle)) ) {
            // Speed limit is cos(angle)
            return Math.cos(angle);
        } else {
            // We're below speed limit
            return a;
        }

        // This was the old way
        // double c = a / (Math.sqrt(a * a + (b * b)) + Constants.EPSILON);
        // c = (c + (a * 2)) / 3.0;
        // return c;
    }

    /** Create translation, rotation and debug joysticks */
    private void createJoysticks() {

        // Create translation joystick
        if (JoystickMap.JOYSTICK_TRANSLATION_USB > JoystickMap.NOT_IN_USE) {
            _translation = new Joystick(JoystickMap.JOYSTICK_TRANSLATION_USB);
            _xAxis = JoystickMap.JOYSTICK_TRANSLATION.X_AXIS;
            _xSign = JoystickMap.JOYSTICK_TRANSLATION.X_SIGN;
            _yAxis = JoystickMap.JOYSTICK_TRANSLATION.Y_AXIS;
            _ySign = JoystickMap.JOYSTICK_TRANSLATION.Y_SIGN;
            _translationButtons = JoystickMap.JOYSTICK_TRANSLATION.BUTTONS;
        } else if (JoystickMap.GAMEPAD_TRANSLATION_USB > JoystickMap.NOT_IN_USE) {
            _translation = new Gamepad(JoystickMap.GAMEPAD_TRANSLATION_USB);
            _xAxis = JoystickMap.GAMEPAD_TRANSLATION.X_AXIS;
            _xSign = JoystickMap.GAMEPAD_TRANSLATION.X_SIGN;
            _yAxis = JoystickMap.GAMEPAD_TRANSLATION.Y_AXIS;
            _ySign = JoystickMap.GAMEPAD_TRANSLATION.Y_SIGN;
            _translationButtons = JoystickMap.GAMEPAD_TRANSLATION.BUTTONS;
        }

        // Create rotation joystick
        if (JoystickMap.JOYSTICK_ROTATION_USB > JoystickMap.NOT_IN_USE) {
            _rotation = new Joystick(JoystickMap.JOYSTICK_ROTATION_USB);
            _twistAxis = JoystickMap.JOYSTICK_ROTATION.TWIST_AXIS;
            _twistSign = JoystickMap.JOYSTICK_ROTATION.TWIST_SIGN;
            _rotationButtons = JoystickMap.JOYSTICK_ROTATION.BUTTONS;
        } else if (JoystickMap.GAMEPAD_ROTATION_USB > JoystickMap.NOT_IN_USE) {
            _rotation = new Gamepad(JoystickMap.GAMEPAD_ROTATION_USB);
            _twistAxis = JoystickMap.GAMEPAD_ROTATION.TWIST_AXIS;
            _twistSign = JoystickMap.GAMEPAD_ROTATION.TWIST_SIGN;
            _rotationButtons = JoystickMap.GAMEPAD_ROTATION.BUTTONS;
        }

        // Create debug joystick
        if (JoystickMap.JOYSTICK_DEBUG_USB > JoystickMap.NOT_IN_USE) {
            _debug = new Joystick(JoystickMap.JOYSTICK_DEBUG_USB);
            _debugButtons = JoystickMap.JOYSTICK_DEBUG.BUTTONS;
        } else if (JoystickMap.GAMEPAD_DEBUG_USB > JoystickMap.NOT_IN_USE) {
            _debug = new Gamepad(JoystickMap.GAMEPAD_DEBUG_USB);
            _debugButtons = JoystickMap.GAMEPAD_DEBUG.BUTTONS;
        }

    }

    /** Create translation, rotation and debug joystick buttons */
    private void createButtons() {
        
        // Create translation joystick buttons
        for (int i = 0; i < _translationButtons.length; i++) {
            addJoystickButton(_translation, i+1, _translationButtons[i]);
        }

        // Create rotation joystick buttons
        for (int i = 0; i < _rotationButtons.length; i++) {
            addJoystickButton(_rotation, i+1, _rotationButtons[i]);
        }

        // Create debug joystick buttons
        for (int i = 0; i < _debugButtons.length; i++) {
            addJoystickButton(_debug, i+1, _debugButtons[i]);
        }

    }

    @Override
    public void updateDashboard() {}

}
