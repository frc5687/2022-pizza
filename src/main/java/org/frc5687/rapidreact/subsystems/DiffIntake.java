package org.frc5687.rapidreact.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.frc5687.rapidreact.util.OutliersContainer;

public class DiffIntake extends OutliersSubsystem{

    private TalonFX talon;
    private float speed;

    public DiffIntake(OutliersContainer container) {
        super(container);
        talon = new TalonFX(9);
        metric("Debug1", true);
    }

    public void spin(float demand){
        talon.set(ControlMode.PercentOutput, demand);
        metric("Debug2", true);
    }

    @Override
    public void updateDashboard() {
    }
}
