package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.util.OutliersContainer;

/** Intake balls onto robot. */
public class Intake extends OutliersSubsystem {

    public enum IntakeState {
        STOWED, // stowed on robot, not spinning
        DEPLOYED_SPIN_IN, // deployed, spinning to pick up balls
        DEPLOYED_NO_SPIN, // deployed, not spinning
        DEPLOYED_SPIN_OUT, // deployed, spinning to push away balls
        ERROR // something strange happened, do no harm
    }
    private IntakeState _state;

    // Note: once intake gets in ERROR state it can't get out except with
    // an explicit setState().
    // Check for ERROR state in every action method.

    /** Create an intake subsystem
     * 
     * @param container
     */
    public Intake(OutliersContainer container) {
        super(container);
        _state = IntakeState.STOWED;
        updateDashboard();
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

    // Actions

    public void stowe() {
        // Stowe the intake
        switch (_state) {
            case ERROR:
                doNoHarm(); // do no harm in ERROR state
                return;
            case DEPLOYED_SPIN_IN:
            case DEPLOYED_SPIN_OUT:
                spinStop();
            default:
                break;
        }
        // Take action to stowe the intake (e.g. pneumatics)
        // Check that intake is stowed (e.g. Hall effect sensor)
        // If everything checks out, change state of intake
        _state = IntakeState.STOWED;
        updateDashboard();
    }

    public void deploy() {
        //Deploy the intake
        switch (_state) {
            case ERROR:
                doNoHarm(); // do no harm in ERROR state
                return;
            case STOWED:
                // We're good to deploy
                // Take action to deploy the intake (e.g. pneumatics)
                // Check that intake is deployed (e.g. Hall effect sensor)
                // If everything checks out, change state of intake
                _state = IntakeState.DEPLOYED_NO_SPIN;
                break;
            default:
                // Whoa, we're not stowed
                // Maybe no big deal to try to deploy when we're not stowed,
                // so could ignore it.
                // But if we think this is an error condition, could change
                // state.
                // For now, let's flag it as an error if we try to deploy()
                // when we're not STOWED.
                _state = IntakeState.ERROR;
                break;
        }
        updateDashboard();
    }

    public void spinIn() {
        // Spin the intake to pick up balls
        switch (_state) {
            case ERROR:
                doNoHarm(); // do no harm in ERROR state
                return;
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
        updateDashboard();
    }

    public void spinOut() {
        // Spin the intake to push away balls
        switch (_state) {
            case ERROR:
                doNoHarm(); // do no harm in ERROR state
                return;
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
        updateDashboard();
    }

    public void spinStop() {
        // Stop the intake from spinning
        switch (_state) {
            case ERROR:
                doNoHarm(); // do no harm in ERROR state
                return;
            case DEPLOYED_SPIN_IN:
            case DEPLOYED_SPIN_OUT:
                stopSpinning();
                _state = IntakeState.DEPLOYED_NO_SPIN;
            default:
                break;
        }
        updateDashboard();
    }

    // Helper methods

    @Override
    public void updateDashboard() {
        // Show intake state on dashboard
        metric("Intake State", getState().name());
    }

    /** Allow doNoHarm to stop rollers from spinning */
    private void stopSpinning() {
        // Take action to stop rollers from spinning
    }

    private void doNoHarm() {
        // Do no harm, intake is in ERROR state
        // Cut off current to motors, etc.
        stopSpinning();
    }
}
