package frc.team4276.frc2024.shooting;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team4276.frc2024.RobotState;

public class FerryUtil {
    //TODO: borrow Citrus' homework
    public static double[] getFerryParams(Pose2d robot_pose){
        Translation2d robot_to_target = RobotState.getInstance().getPOIs().kBank
            .translateBy(robot_pose.getTranslation().inverse());

        double[] params = {0.0, 0.0, 0.0};

        return params;

    }
}
