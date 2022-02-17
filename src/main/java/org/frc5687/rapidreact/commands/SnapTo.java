package org.frc5687.rapidreact.commands;


import org.frc5687.rapidreact.subsystems.DriveTrain;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

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
        _driveTrain.snap(_theta);
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return _driveTrain.isAtRotation(_theta);
        //return true;
    }
}
