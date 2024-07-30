package frc.team4276.frc2024.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2024.subsystems.Superstructure;

public class SuperstructureAction implements Action {
    protected final Superstructure mSuperstructure = Superstructure.getInstance();
    protected Superstructure.GoalState mDesiredState;

    protected double mTimeout = -1.0;
    private double mStartTime = 0.0;

    public SuperstructureAction(Superstructure.GoalState goal) {
        mDesiredState = goal;
    }

    public SuperstructureAction(Superstructure.GoalState goal, double timeout) {
        mDesiredState = goal;
        this.mTimeout = timeout;
    }

    @Override
    public void start() {
        this.mStartTime = Timer.getFPGATimestamp();
        mSuperstructure.setGoalState(mDesiredState);
    }

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        if (mTimeout >= 0 && Timer.getFPGATimestamp() - mStartTime >= mTimeout) return true;
        return mDesiredState == Superstructure.GoalState.SHOOT && !mSuperstructure.isHoldingNote();
    }

    @Override
    public void done() {}
}
