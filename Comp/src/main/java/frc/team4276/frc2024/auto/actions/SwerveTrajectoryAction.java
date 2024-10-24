package frc.team4276.frc2024.auto.actions;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;

import frc.team254.lib.geometry.Pose2d;
import frc.team4276.frc2024.field.AllianceChooser;

public class SwerveTrajectoryAction implements Action {
    private Command command;
    private Pose2d initial;

    public SwerveTrajectoryAction(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);

            var p = AllianceChooser.getInstance().isAllianceRed() ? path.flipPath() : path;

            initial = Pose2d.fromWPI(p.getStartingDifferentialPose());
                
            command = AutoBuilder.followPath(path);

        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

    }

        public SwerveTrajectoryAction(String name, int split) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name, split);

            var p = AllianceChooser.getInstance().isAllianceRed() ? path.flipPath() : path;

            initial = Pose2d.fromWPI(p.getStartingDifferentialPose());
                
            command = AutoBuilder.followPath(path);

        } catch (Exception e) {
            System.out.println(e.getMessage());
        }

    }

    @Override
    public void start() {
        command.initialize();
    }

    @Override
    public void update() {
        command.execute();
    }

    @Override
    public boolean isFinished() {
        return command.isFinished();
    }

    @Override
    public void done() {
        command.end(false);
    }

    public Pose2d getInitialPose() {
        return initial;
    }
}
