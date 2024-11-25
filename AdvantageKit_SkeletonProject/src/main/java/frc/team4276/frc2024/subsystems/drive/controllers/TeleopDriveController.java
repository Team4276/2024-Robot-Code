package frc.team4276.frc2024.subsystems.drive.controllers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.subsystems.drive.DriveConstants;

public class TeleopDriveController {
    private final double controllerDeadband = 0.1;
    private final double angularVelocityScalar = 0.5;
    private final double velocityScalar = 1.0;

    private double controllerX = 0.0;
    private double controllerY = 0.0;
    private double controllerOmega = 0.0;

    public void feedDriveInput(double x, double y, double omega){
        controllerX = x;
        controllerY = y;
        controllerOmega = omega;
    }

    public ChassisSpeeds update(Rotation2d heading){
        if(Math.hypot(controllerX, controllerY) < controllerDeadband) {
            return new ChassisSpeeds();
        }

        return ChassisSpeeds.fromFieldRelativeSpeeds(
            controllerX * DriveConstants.kMaxVel * velocityScalar,
            controllerY * DriveConstants.kMaxVel * velocityScalar,
            controllerOmega * DriveConstants.kMaxAngularVel * velocityScalar * angularVelocityScalar,
            RobotState.getInstance().getLatestFieldToVehicle().getRotation().rotateBy(
                DriverStation.getAlliance().get() == DriverStation.Alliance.Red ?
                Rotation2d.fromDegrees(180.0) : new Rotation2d()));
    }
}
