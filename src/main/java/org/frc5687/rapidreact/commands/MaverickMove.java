package org.frc5687.rapidreact.commands;

import org.frc5687.rapidreact.subsystems.Maverick;

import edu.wpi.first.math.geometry.Pose2d;

public class MaverickMove extends OutliersCommand{
    
    private Maverick _maverick;
    
    public MaverickMove(Maverick maverick){
        _maverick = maverick;
    }

    @Override
    public void initialize(){
        super.initialize();
    }

    @Override
    public void execute(){
        super.execute();
        metric("Maverick", true);
        _maverick.wayPointMove();
    }

    @Override
    public boolean isFinished(){
        return _maverick.isAtPose();
    }
}