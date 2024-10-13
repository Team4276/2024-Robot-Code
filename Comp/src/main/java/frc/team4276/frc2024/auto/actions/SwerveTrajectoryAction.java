package frc.team4276.frc2024.auto.actions;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;

import frc.team254.lib.geometry.Pose2d;

public class SwerveTrajectoryAction implements Action {
    private Command command;
    private Pose2d initial;

    /**
     * Alliance Test
     */
    public SwerveTrajectoryAction(String name) {
        try {
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(name);

            initial = Pose2d.fromWPI(path.getStartingDifferentialPose());
                
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
