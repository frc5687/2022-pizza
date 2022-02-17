package org.frc5687.rapidreact.commands;
import org.frc5687.rapidreact.subsystems.DriveTrain;
import org.frc5687.rapidreact.util.Limelight;

public class Aim extends OutliersCommand{

    private DriveTrain _driveTrain;
    private Limelight _limelight;
    
    public Aim(DriveTrain driveTrain, Limelight limelight){
        _driveTrain = driveTrain;
        _limelight = limelight;
    }

    @Override
    public void execute(){
        super.execute();
        error("Aiming");
        _driveTrain.useAutoAim(true);
    }
}
