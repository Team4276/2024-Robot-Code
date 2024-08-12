package frc.team4276.lib.characterizations;

import edu.wpi.first.math.controller.ElevatorFeedforward;

public class ElevatorFeedForward extends ElevatorFeedforward implements IFeedForward{
    public ElevatorFeedForward(double ks, double kg, double kv, double ka){
        super(ks, kg, kv, ka);
    }

    public ElevatorFeedForward(double ks, double kg, double kv){
        super(ks, kg, kv);
    }

    @Override
    public double calculate(double pos, double vel, double accel) {
        return super.calculate(vel, accel);
    }

    @Override
    public boolean isLinear() {
        return true;
    }
}