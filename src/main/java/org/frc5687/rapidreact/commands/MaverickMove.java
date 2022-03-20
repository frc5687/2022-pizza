package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Maverick;

public class MaverickMove extends OutliersCommand{
    
    private Maverick _maverick;
    
    public MaverickMove(Maverick maverick){
        _maverick = maverick;
        metric("Maverick Init", true);
    }

    @Override
    public void initialize(){
        super.initialize();
        _maverick.rumble();
    }

    @Override
    public void execute(){
        super.execute();
        metric("Hello", true);
        _maverick.wayPointMove();
    }

    @Override
    public boolean isFinished(){
        return _maverick.isAtPose();
    }

    @Override
    public void end(boolean interrupted){
        super.end(interrupted);
        _maverick.stopRumble();
    }
}