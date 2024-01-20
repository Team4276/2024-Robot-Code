package frc.team4276.lib.drivers;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.team4276.frc2024.Constants;

// Modified Version of 1678s 2023 Pigeon class

public class Pigeon {

    private static Pigeon mInstance;

    public static Pigeon getInstance() {
        if (mInstance == null) {
            mInstance = new Pigeon();
        }
        return mInstance;
    }

    // Actual pigeon object
    private final ADIS16470_IMU mGyro;

    // Configs
    private boolean inverted = Constants.DriveConstants.kGyroReversed;
    private Rotation2d yawAdjustmentAngle = new Rotation2d();
    private Rotation2d rollAdjustmentAngle = new Rotation2d();
    private Rotation2d pitchAdjustmentAngle = new Rotation2d();

    private Pigeon() {        
        mGyro = new ADIS16470_IMU();
    }

    public Rotation2d getYaw() {
        Rotation2d angle = getUnadjustedYaw().rotateBy(yawAdjustmentAngle.unaryMinus());
        if (inverted) {
            return angle.unaryMinus();
        }
        return angle;
    }

    public Rotation2d getRoll() {
        return getUnadjustedRoll().rotateBy(rollAdjustmentAngle.unaryMinus());
    }

    public Rotation2d getPitch() {
        return getUnadjustedPitch().rotateBy(pitchAdjustmentAngle.unaryMinus()).unaryMinus();
    }

    /**
     * Sets the yaw register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setYaw(double angleDeg) {
        yawAdjustmentAngle = getUnadjustedYaw().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    /**
     * Sets the roll register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setRoll(double angleDeg) {
        rollAdjustmentAngle = getUnadjustedRoll().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    /**
     * Sets the pitch register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setPitch(double angleDeg) {
        pitchAdjustmentAngle = getUnadjustedPitch().rotateBy(Rotation2d.fromDegrees(angleDeg).unaryMinus());
    }

    public Rotation2d getUnadjustedYaw() {
        return Rotation2d.fromDegrees(mGyro.getAngle(mGyro.getYawAxis()));
    }

    public Rotation2d getUnadjustedPitch() {
        return Rotation2d.fromDegrees(mGyro.getYComplementaryAngle());
    }

    public Rotation2d getUnadjustedRoll() {
        return Rotation2d.fromDegrees(mGyro.getXComplementaryAngle());
    }
}
