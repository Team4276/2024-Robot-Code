package frc.team4276.frc2024.Limelight;

import frc.team1678.lib.swerve.ChassisSpeeds;
import frc.team254.lib.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;


public class VisionPoseAcceptor {
    public boolean shouldAcceptVision(Pose2d vehicleToTag, ChassisSpeeds speeds) {
        if (DriverStation.isAutonomous()) {
            double kMaxRange = 4.0;
            if (vehicleToTag.getTranslation().norm() > kMaxRange) {
                return false;
            }
        }

        boolean rotatingTooFast = Math.abs(speeds.omegaRadiansPerSecond) >= 1.0;
        if (rotatingTooFast) {
            return false;
        }

        return true;
    }

}