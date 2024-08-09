package frc.team4276.frc2024.shooting;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team4276.frc2024.RobotState;

public class FerryUtil {
    public static double[] getFerryParams(Pose2d robot_pose) {
        Translation2d robot_to_target = RobotState.getInstance().getPOIs().kBank
            .translateBy(robot_pose.getTranslation().inverse());

        double flywheel_setpoint = RegressionMaps.kFerryFlywheelRPMs.get(robot_to_target.norm());
        double fourbar_setpoint = RegressionMaps.kFerryFourbarAngles.get(robot_to_target.norm());
        double heading_setpoint = robot_to_target.direction().getRadians();

        return new double[] { flywheel_setpoint, fourbar_setpoint, heading_setpoint };
    }
}
