package frc.team4276.lib.characterizations;

public interface IFeedForward {
    
    public double calculate(double pos, double vel, double accel);

    /**
     * @return false if the characterization uses radians
     */
    public boolean isLinear();
    
}
