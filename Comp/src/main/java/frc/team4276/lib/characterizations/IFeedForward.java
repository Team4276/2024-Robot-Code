package frc.team4276.lib.characterizations;

//TODO: implement common feed forwards
public interface IFeedForward {
    
    public double calculate(double pos, double vel, double accel);

    public double calculate(double pos, double vel);
    
}
