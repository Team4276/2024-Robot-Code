package frc.team1678.lib.swerve;

import frc.team1678.lib.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class ModuleState extends SwerveModulePosition {

    public double speedMetersPerSecond;

    public ModuleState() {
        super(0.0, new Rotation2d());
        speedMetersPerSecond = 0.0;
    }

    public ModuleState(double distanceMeters, Rotation2d angle, double speedMetersPerSecond) {
        super(distanceMeters, angle);
        this.speedMetersPerSecond = speedMetersPerSecond;
    }

    public static ModuleState fromSpeeds(Rotation2d angle, double speedMetersPerSecond) {
        return new ModuleState(Double.NaN, angle, speedMetersPerSecond);
    }

    public static ModuleState optimize(Rotation2d desiredAngle, ModuleState currentState) {
        double targetAngle = Util.placeInAppropriate0To360Scope(desiredAngle.getDegrees(), currentState.angle.getDegrees());
        double targetSpeed = currentState.speedMetersPerSecond;
        double delta = targetAngle - desiredAngle.getDegrees();
        if (Math.abs(delta) > 90){
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }        
        return ModuleState.fromSpeeds(Rotation2d.fromDegrees(targetAngle), targetSpeed);
      }
}
