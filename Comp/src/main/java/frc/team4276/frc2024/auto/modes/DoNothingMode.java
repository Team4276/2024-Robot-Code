package frc.team4276.frc2024.auto.modes;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;

import frc.team254.lib.geometry.Pose2d;

public class DoNothingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {}

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}