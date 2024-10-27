package frc.team4276.frc2024.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class SuperstructureAction implements Action {
    private final GoalState state;
    private final double timeout;
    private double startTime = 0.0;
    
    public SuperstructureAction(GoalState state, double timeout){
        this.state = state;
        this.timeout = timeout;
    }

    public SuperstructureAction(GoalState state){
        this(state, -1);
    }

    @Override
    public void start() {
        Superstructure.getInstance().setGoalState(state);
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return startTime + timeout <= Timer.getFPGATimestamp();
    }

    @Override
    public void done() {
    }
}
