package org.frc5687.rapidreact.commands.auto;

import org.frc5687.rapidreact.commands.OutliersCommand;
import edu.wpi.first.wpilibj.Timer;

/**
 * Wait (do nothing) for a number of seconds so we can pause during auto.
 */
public class AutoWait extends OutliersCommand {

    private double _delay; // Seconds to wait
    private Timer _timer;

    /** constructor */
    public AutoWait(double delay) {
        _delay = delay;
        _timer = new Timer();
    }
   
    @Override
    public void initialize() {
        super.initialize();
        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute() {
        super.execute();
        // do nothing
    }

    @Override
    public boolean isFinished() {
        return (_timer.get() >= _delay);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _timer.reset();
    }
 
}
