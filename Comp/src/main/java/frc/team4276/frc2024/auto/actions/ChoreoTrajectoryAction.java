package frc.team4276.frc2024.auto.actions;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;

import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.subsystems.DriveSubsystem;

public class ChoreoTrajectoryAction implements Action {
    private Pose2d initial = Pose2d.identity();
    private boolean isInit = false;

    private Trajectory<SwerveSample> traj;

    @SuppressWarnings("unchecked")
    public ChoreoTrajectoryAction(String name) {
        try {
            var t = Choreo.loadTrajectory(name);

            if (t.isEmpty())
                return;

            traj = (Trajectory<SwerveSample>) t.get();

            if (AllianceChooser.getInstance().isAllianceRed()) {
                traj = traj.flipped();

            }

            initial = Pose2d.fromWPI(traj.getInitialPose(false));

            isInit = true;

        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

    }
    
    @SuppressWarnings("unchecked")
    public ChoreoTrajectoryAction(String name, int split) {
        try {
            var t = Choreo.loadTrajectory(name);

            if (t.isEmpty())
                return;

            var s = t.get().getSplit(split);

            if (s.isEmpty())
                return;

            traj = (Trajectory<SwerveSample>) s.get();

            if (AllianceChooser.getInstance().isAllianceRed()) {
                traj = traj.flipped();

            }

            initial = Pose2d.fromWPI(traj.getInitialPose(false));

            isInit = true;

        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

    }

    @Override
    public void start() {
        if (!isInit)
            return;

        DriveSubsystem.getInstance().setChoreoTraj(traj);
    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        return !isInit || DriveSubsystem.getInstance().isTrajFinished();
    }

    @Override
    public void done() {
    }

    public Pose2d getInitialPose() {
        return initial;
    }
}
