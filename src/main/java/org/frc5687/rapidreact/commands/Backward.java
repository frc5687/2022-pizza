package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.DiffIntake;

public class Backward extends OutliersCommand{
    
    private DiffIntake _diffIntake;

    public Backward(DiffIntake diffIntake){
        _diffIntake = diffIntake; 
    }

    @Override
    public void execute(){
        _diffIntake.spin(-10);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        _diffIntake.spin(0);
    }
}
