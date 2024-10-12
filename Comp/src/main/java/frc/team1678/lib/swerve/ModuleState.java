package frc.team1678.lib.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

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
        var delta = desiredState.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
        return ModuleState.fromSpeeds(
            desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)),
            -desiredState.speedMetersPerSecond
            );
        } else {
            return ModuleState.fromSpeeds(desiredState.angle, desiredState.speedMetersPerSecond);
        }
    }
}
