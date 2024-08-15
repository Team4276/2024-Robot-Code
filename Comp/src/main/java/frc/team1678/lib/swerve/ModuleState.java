package frc.team1678.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.team1678.lib.Util;

public class ModuleState extends SwerveModulePosition {
    protected static final ModuleState kIdentity = new ModuleState();

    public static ModuleState identity() {
        return kIdentity;
    }

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

    public static ModuleState optimize(ModuleState desiredState, Rotation2d currentAngle) {
        double currentAngleD = currentAngle.getDegrees();
        double targetAngle = Util.placeInAppropriate0To360Scope(currentAngleD, desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = currentAngleD - targetAngle;
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return ModuleState.fromSpeeds(Rotation2d.fromDegrees(targetAngle), targetSpeed);
    }
}
