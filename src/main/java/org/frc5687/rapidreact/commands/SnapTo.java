package org.frc5687.rapidreact.commands;

// import org.frc5687.rapidreact.Constants.SnapPose;
import org.frc5687.rapidreact.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;

public class SnapTo extends OutliersCommand{
    
    private DriveTrain _driveTrain;
    private Rotation2d _theta;
    
    public SnapTo(DriveTrain driveTrain, Rotation2d theta){
        _driveTrain = driveTrain;
        _theta = theta;
        addRequirements(_driveTrain);
    }

    @Override
    public void execute(){
        super.execute();
    }

    @Override
    public void initialize(){
        super.initialize();
        _driveTrain.snap(_theta);
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return _driveTrain.isAtRotation(_theta);
    }
}
