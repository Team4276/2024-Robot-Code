package frc.team4276.frc2024.shooting;

import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;

public class ShootingUtil {
    public static double[] getSpeakerShotParams(Pose2d robot_pose){
        Translation2d robot_to_target = RobotState.getInstance().getPOIs().kSpeakerCenter
            .translateBy(robot_pose.getTranslation().inverse());

        double distance = robot_to_target.norm();
        double flywheel_setpoint = RegressionMaps.kSpeakerFlywheelRPMs.get(robot_to_target.norm());
        double fourbar_setpoint = RegressionMaps.kSpeakerFourbarAngles.get(robot_to_target.norm());
        Rotation2d heading_setpoint = AllianceChooser.getInstance().isAllianceRed() ?
            robot_to_target.direction() :
            robot_to_target.direction().rotateBy(Rotation2d.fromDegrees(180));

        return new double[] { distance, flywheel_setpoint, fourbar_setpoint, heading_setpoint.getRadians()};

    }
    
}
