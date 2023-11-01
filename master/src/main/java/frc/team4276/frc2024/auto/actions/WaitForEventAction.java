package frc.team4276.frc2024.auto.actions;

import frc.team4276.frc2024.auto.AutoEvents;

public class WaitForEventAction implements Action{
    @Override
    public void start() {}

    @Override
    public void update() {}

    @Override
    public boolean isFinished() {
        return AutoEvents.event;
    }

    @Override
    public void done() {}
}