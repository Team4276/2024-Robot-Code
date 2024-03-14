package frc.team4276.frc2024.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;

import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;

public class DoNothingMode extends AutoModeBase {
    @Override
    protected void routine() throws AutoModeEndedException {}

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}