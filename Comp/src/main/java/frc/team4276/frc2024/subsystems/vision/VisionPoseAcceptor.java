package frc.team4276.frc2024.subsystems.vision;

import frc.team1678.lib.swerve.ChassisSpeeds;

//TODO: calibrate
public class VisionPoseAcceptor {
    public boolean shouldAcceptVision(ChassisSpeeds speeds) {
        if (Math.abs(speeds.omegaRadiansPerSecond) >= 1.0) {
            return false;
        }

        return true;
    }

}