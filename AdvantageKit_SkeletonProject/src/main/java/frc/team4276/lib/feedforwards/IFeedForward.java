package frc.team4276.lib.feedforwards;

public interface IFeedForward {
    
    public double calculate(double pos, double vel, double accel);

    /**
     * @return false if the characterization uses radians
     */
    public boolean isLinear();
    
}
