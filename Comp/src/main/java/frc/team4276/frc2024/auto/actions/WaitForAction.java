package frc.team4276.frc2024.auto.actions;

import java.util.function.BooleanSupplier;

public class WaitForAction implements Action {
    private final BooleanSupplier s;

    public WaitForAction(BooleanSupplier s){
        this.s = s;
    }

    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return s.getAsBoolean();
    }

    @Override
    public void done() {}
    
}
