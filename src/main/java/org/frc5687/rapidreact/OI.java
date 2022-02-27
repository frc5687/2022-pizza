/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * Has some instructions on how to switch controls.
 * See JoysticknMap for configuration of joystick and gamepad.
*/
package org.frc5687.rapidreact;

import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.frc5687.rapidreact.subsystems.Catapult;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Indexer;
import org.frc5687.rapidreact.subsystems.Intake;

// import org.frc5687.rapidreact.commands.ResetNavX;

// import org.frc5687.rapidreact.util.AxisButton;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;
import static org.frc5687.rapidreact.util.Helpers.*;

import org.frc5687.rapidreact.config.JoystickMap;
import org.frc5687.rapidreact.config.Constants;

 /** Operator input
 * 
 * <p> Define the types of devices that can provide operator input
 * (joysticks, gamepads, drive wheels, keyboards, etc.)
 * 
 * @see JoystickMap
 */
public class OI extends OutliersProxy {

    // Declare joysticks
    private Joystick _translation;
    private Joystick _rotation;
    private Joystick _debug;

    // Declare joystick axes
    private int _xAxis;
    private int _yAxis;
    private int _twistAxis;

    /** Commands that can be mapped to buttons */
    public static enum Command {
        NOT_IN_USE,
        DEPLOY_INTAKE,
        RETRACT_INTAKE,
        RESET_NAVX,
        RUN_AUTO,
        SHOOT
    }    

    // Declare joystick buttons
    private JoystickButton _deployIntake;
    private JoystickButton _resetNavX;
    private JoystickButton _retractIntake;
    private JoystickButton _runAuto;
    private JoystickButton _shoot;

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
            case RESET_NAVX:
                _resetNavX = new JoystickButton(joystick, buttonNumber);
                break;
            case RUN_AUTO:
                _runAuto = new JoystickButton(joystick, buttonNumber);
                break;
            case SHOOT:
                _shoot = new JoystickButton(joystick, buttonNumber);
                break;
        }
    }

    /** Define how buttons work
     * 
     * <ul>
     *  <li> when button calls command (when pressed, released, held, etc.)
     *  <li> which command gets called
     * </ul>
     */
    public void initializeButtons(
        Catapult catapult,
        DriveTrain driveTrain,
        Indexer indexer,
        Intake intake
        ) {
        _resetNavX.whenReleased(new InstantCommand(driveTrain::resetYaw, driveTrain));
        _shoot.whenPressed(new InstantCommand(catapult::shoot, catapult));
    }

    /** Get X value from translation joystick */
    public double getDriveX() {
        // TODO: explain the negative sign here
        xIn = -getSpeedFromAxis(_translation, _xAxis);
        xIn = applyDeadband(xIn, Constants.DriveTrain.DEADBAND_TRANSLATION);
        return circularize(xIn, yIn);
    }

    /** Get Y value from translation joystick */
    public double getDriveY() {
        yIn = getSpeedFromAxis(_translation, _yAxis);
        yIn = applyDeadband(yIn, Constants.DriveTrain.DEADBAND_TRANSLATION);
        return circularize(yIn, xIn);
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

    /** Get rotation value from rotation joystick */
    public double getRotationX() {
        double speed = getSpeedFromAxis(_rotation, _twistAxis);
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND_ROTATION);
        return speed;
    }

    protected double getSpeedFromAxis(Joystick joystick, int axisNumber) {
        return joystick.getRawAxis(axisNumber);
    }

    // Helper methods

    /** Create translation, rotation and debug joysticks */
    private void createJoysticks() {

        // Create translation joystick
        if (JoystickMap.JOYSTICK_TRANSLATION_USB > JoystickMap.NOT_IN_USE) {
            _translation = new Joystick(JoystickMap.JOYSTICK_TRANSLATION_USB);
            _xAxis = JoystickMap.JOYSTICK_TRANSLATION.X;
            _yAxis = JoystickMap.JOYSTICK_TRANSLATION.Y;
            _translationButtons = JoystickMap.JOYSTICK_TRANSLATION.BUTTONS;
        } else if (JoystickMap.GAMEPAD_TRANSLATION_USB > JoystickMap.NOT_IN_USE) {
            _translation = new Gamepad(JoystickMap.GAMEPAD_TRANSLATION_USB);
            _xAxis = JoystickMap.GAMEPAD_TRANSLATION.X;
            _yAxis = JoystickMap.GAMEPAD_TRANSLATION.Y;
            _translationButtons = JoystickMap.GAMEPAD_TRANSLATION.BUTTONS;
        }

        // Create rotation joystick
        if (JoystickMap.JOYSTICK_ROTATION_USB > JoystickMap.NOT_IN_USE) {
            _rotation = new Joystick(JoystickMap.JOYSTICK_ROTATION_USB);
            _twistAxis = JoystickMap.JOYSTICK_ROTATION.Twist;
            _rotationButtons = JoystickMap.JOYSTICK_ROTATION.BUTTONS;
        } else if (JoystickMap.GAMEPAD_ROTATION_USB > JoystickMap.NOT_IN_USE) {
            _rotation = new Gamepad(JoystickMap.GAMEPAD_ROTATION_USB);
            _twistAxis = JoystickMap.GAMEPAD_ROTATION.Twist;
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
        for (int i = 1; i < (_translationButtons.length + 1); i++) {
            addJoystickButton(_translation, i, _translationButtons[i]);
        }

        // Create rotation joystick buttons
        for (int i = 1; i < (_rotationButtons.length + 1); i++) {
            addJoystickButton(_rotation, i, _rotationButtons[i]);
        }

        // Create debug joystick buttons
        for (int i = 1; i < (_debugButtons.length + 1); i++) {
            addJoystickButton(_debug, i, _debugButtons[i]);
        }

    }

    @Override
    public void updateDashboard() {}

}
