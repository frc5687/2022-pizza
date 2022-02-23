package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.util.OutliersContainer;

/**
 * Intake balls onto robot.
 */
public class Intake extends OutliersSubsystem{

    public enum IntakeState {
        STOWED, // stowed on robot, not spinning
        DEPLOYED_SPIN_IN, // deployed, spinning to pick up balls
        DEPLOYED_NO_SPIN, // deployed, not spinning
        DEPLOYED_SPIN_OUT, // deployed, spinning to push away balls
        ERROR // trying to do something strange
    }
    private IntakeState _state;

    /** Constructor */
    public Intake(OutliersContainer container) {
        //Construct the roller and solenoids
        super(container);
        _state = IntakeState.STOWED;
    }

    // Query state of intake

    public IntakeState getState() {
        return _state;
    }

    public Boolean isStowed() {
        return (_state == IntakeState.STOWED);
    }

    public Boolean isDeployed() {
        switch (_state) {
            case DEPLOYED_SPIN_IN:
            case DEPLOYED_NO_SPIN:
            case DEPLOYED_SPIN_OUT:
                return true;
            default:
                return false;
        }
    }

    // Set state of intake

    public void setState(IntakeState state) {
        _state = state;
    }

    public void stowe() {
        // Stowe the intake
        switch (_state) {
            case DEPLOYED_SPIN_IN:
            case DEPLOYED_SPIN_OUT:
                spinStop();
            default:
                break;
        }
        _state = IntakeState.STOWED;
    }

    public void deploy() {
        //Deploy the intake
        if (_state == IntakeState.STOWED) {
            _state = IntakeState.DEPLOYED_NO_SPIN;
        } else {
            _state = IntakeState.ERROR;
        }
    }

    public void spinIn() {
        // Spin the intake to pick up balls
        switch (_state) {
            case DEPLOYED_NO_SPIN:
                _state = IntakeState.DEPLOYED_SPIN_IN;
                break;
            case DEPLOYED_SPIN_OUT:
                spinStop();
                _state = IntakeState.DEPLOYED_SPIN_IN;
                break;
            default:
                break;
        }
    }

    public void spinOut() {
        // Spin the intake to push away balls
        switch (_state) {
            case DEPLOYED_NO_SPIN:
                _state = IntakeState.DEPLOYED_SPIN_OUT;
                break;
            case DEPLOYED_SPIN_IN:
                spinStop();
                _state = IntakeState.DEPLOYED_SPIN_OUT;
                break;
            default:
                break;
        }
    }

    public void spinStop() {
        // Stop the intake from spinning
        switch (_state) {
            case DEPLOYED_SPIN_IN:
            case DEPLOYED_SPIN_OUT:
                _state = IntakeState.DEPLOYED_NO_SPIN;
            default:
                break;
        }
    }

    @Override
    public void updateDashboard() {
        // Intake state
        metric("Intake State", getState().name());
    }
}
