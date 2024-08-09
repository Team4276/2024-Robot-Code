package frc.team4276.frc2024.auto.actions;

import frc.team4276.frc2024.RobotState;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Translation2d;

public class WaitForRegionAction implements Action {
    private Translation2d mBottomLeft;
    private Translation2d mTopRight;

    public WaitForRegionAction(Translation2d bottomLeft, Translation2d topRight) {
        mBottomLeft = bottomLeft;
        mTopRight = topRight;
    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
    }

    @Override
    public boolean isFinished() {
        Pose2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle();
        return robot_pose.getTranslation().x() < mTopRight.x() && robot_pose.getTranslation().x() > mBottomLeft.x()
                && robot_pose.getTranslation().y() > mBottomLeft.y() && robot_pose.getTranslation().y() < mTopRight.y();
    }

    @Override
    public void done() {}
}
