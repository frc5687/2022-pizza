/** Team 5687 (C)2021-2022
 * Joystick and gamepad control for the robot.
 * Also has button inits.
 * Has some instructions on how to switch controls.
*/
package org.frc5687.rapidreact;

import static org.frc5687.rapidreact.util.Helpers.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.frc5687.rapidreact.commands.Backward;
import org.frc5687.rapidreact.commands.Forward;
import org.frc5687.rapidreact.commands.MaverickMove;
import org.frc5687.rapidreact.config.Constants;
import org.frc5687.rapidreact.subsystems.DiffIntake;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.subsystems.Intake;
import org.frc5687.rapidreact.subsystems.Maverick;
import org.frc5687.rapidreact.util.Gamepad;
import org.frc5687.rapidreact.util.OutliersProxy;
import edu.wpi.first.wpilibj.GenericHID;

import javax.print.attribute.standard.JobHoldUntil;

public class OI extends OutliersProxy {

    // Joysticks and gamepads
    private Gamepad _debug;
    private Joystick _joy;

    // Buttons
    private JoystickButton _autoAim;
    private JoystickButton _catapultDebugButton;
    private JoystickButton _deployRetract;
    private JoystickButton _dropArm;
    private JoystickButton _exitKill;
    private JoystickButton _intakeButton;
    private JoystickButton _kill;
    private JoystickButton _preloadButton;
    private JoystickButton _readyToClimb;
    private JoystickButton _stowClimber;
    private JoystickButton _release;
    private JoystickButton _shootButton;

    private JoystickButton _maverick;
    private JoystickButton _nextWaypoint;
    private JoystickButton _resetOdometry;
    private JoystickButton _resetNavx;

    private JoystickButton forward;
    private JoystickButton backward;

    private JoystickButton _shootSetpointOne;
    private JoystickButton _shootSetpointTwo;
    private JoystickButton _shootSetpointThree;

    // "Raw" joystick values
    private double yIn = 0;
    private double xIn = 0;

    public OI() {
        _joy = new Joystick(1);
        _debug = new Gamepad(0);

        forward = new JoystickButton(_joy, 5);
        backward = new JoystickButton(_joy, 6);
        _maverick = new JoystickButton(_debug, Gamepad.Buttons.A.getNumber());
        _nextWaypoint = new JoystickButton(_debug, Gamepad.Buttons.B.getNumber());
        _resetOdometry = new JoystickButton(_debug, Gamepad.Buttons.Y.getNumber());
        _resetNavx = new JoystickButton(_debug, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
        
    }

    public void initializeButtons(RobotContainer robotContainer, Maverick maverick, DriveTrain driveTrain, DiffIntake diffIntake) {
        // driving, Ben check pls.
        metric("Init buttons", true);

        forward.whenHeld(new Forward(diffIntake));
        backward.whenHeld(new Backward(diffIntake));
        _nextWaypoint.whenPressed(maverick::nextPoint);
        _resetOdometry.whenPressed(driveTrain::resetOForTesting);
        _resetNavx.whenPressed(driveTrain::resetYaw);
    }

    public boolean readyToClimb() { return _readyToClimb.get(); }
    public boolean isShootButtonPressed() { return _shootButton.get(); }
    public boolean exitDebugCatapult() { return _catapultDebugButton.get(); }
    public boolean preloadCatapult() { return _preloadButton.get(); }
    public boolean releaseArm() { return _release.get(); }
    public boolean intakeDeployRetract() { return _deployRetract.get(); }
    public boolean exitKill() { return _exitKill.get(); }

    public boolean kill() { return _kill.get(); }
    public boolean autoAim() { return _autoAim.get(); }

    public double getDriveY() {
        //Comment for gamepad control
        //rumble();
        yIn = getSpeedFromAxis(_debug, Gamepad.Axes.LEFT_Y.getNumber());
        // yIn = getSpeedFromAxis(Gamepad, Gamepad.getYChannel());
        yIn = applyDeadband(yIn, 0.2);

        double yOut = yIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        yOut = (yOut + (yIn * 2)) / 3.0;
        return yOut;
    }

    public double getDriveX() {
        //Comment for gamepad control
        //rumble();
        xIn = -getSpeedFromAxis(_debug, Gamepad.Axes.LEFT_X.getNumber());
        //xIn = -getSpeedFromAxis(Gamepad, Gamepad.getXChannel());
        xIn = applyDeadband(xIn, 0.2);
        double xOut = xIn / (Math.sqrt(yIn * yIn + (xIn * xIn)) + Constants.EPSILON);
        xOut = (xOut + (xIn * 2)) / 3.0; // numbers from empirical testing.
        return xOut;
    }

    public double getRotationX() {
        double speed = -getSpeedFromAxis(_debug, Gamepad.Axes.RIGHT_X.getNumber());
        speed = applyDeadband(speed, 0.2);
        return speed;
    }

    public void rumble(){
        _debug.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
        _debug.setRumble(GenericHID.RumbleType.kRightRumble, 1);
    }

    public void stopRummble(){
        _debug.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        _debug.setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }

    public double getSpringMotorSpeed() {
        double speed = -getSpeedFromAxis(_debug, Gamepad.Axes.LEFT_Y.getNumber());

        speed = applyDeadband(speed, 0.2);
        return speed;
    }

    public double getWinchMotorSpeed() {
        double speed = -getSpeedFromAxis(_debug, _debug.getXChannel());
        speed = applyDeadband(speed, 0.2);
        return speed;
    }

    public double getStationarySpeed() {
        double speed = -getSpeedFromAxis(_debug, Gamepad.Axes.LEFT_X.getNumber());
        speed = applyDeadband(speed, 0.2);
        speed = 0;
        return speed;
    }
    
    public double getRockerSpeed() {
        double speed = -getSpeedFromAxis(_debug, Gamepad.Axes.RIGHT_Y.getNumber());
        speed = applyDeadband(speed, 0.2);
        speed = 0;
        return speed;
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {

    }
}