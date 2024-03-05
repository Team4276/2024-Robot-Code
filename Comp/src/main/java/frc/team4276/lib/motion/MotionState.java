package frc.team4276.lib.motion;

public class MotionState {
    protected final double time;
    protected final double pos;
    protected final double vel;
    protected final double acc;

    public MotionState(double time, double pos, double vel, double acc){
        this.time = time;
        this.pos = pos;
        this.vel = vel;
        this.acc = acc;
    }
}
