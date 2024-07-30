package frc.team4276.lib.characterizations;

public interface IFeedForward {
    
    public double calculate(double pos, double vel, double accel);

    public double calculate(double pos, double vel);
    
}
