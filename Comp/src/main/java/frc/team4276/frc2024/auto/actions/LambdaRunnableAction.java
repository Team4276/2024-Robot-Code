package frc.team4276.frc2024.auto.actions;

import edu.wpi.first.wpilibj.Timer;

public class LambdaRunnableAction implements Action {
    public interface VoidInterace {
        void f();
    }

    VoidInterace mF;

    protected double mTimeout = -1.0;
    private double mStartTime = 0.0;

    public LambdaRunnableAction(VoidInterace f, double timeout) {
        this.mF = f;
        mTimeout = timeout;
    }

    public LambdaRunnableAction(VoidInterace f) {
        this(f, -1.0);
    }

    @Override
    public void start() {
        mStartTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
        mF.f();
    }

    @Override
    public boolean isFinished() {
        return mTimeout >= 0 && Timer.getFPGATimestamp() - mStartTime >= mTimeout;
    }

    @Override
    public void done() {}
}
