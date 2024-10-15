package frc.team4276.frc2024.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class GyroIOADIS implements GyroIO {
    // Actual pigeon object
    private final ADIS16470_IMU mGyro;

    public GyroIOADIS(){
        mGyro = new ADIS16470_IMU();

    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPosition = getYaw();
        
    }

    // Configs
    private Rotation2d yawAdjustmentAngle = new Rotation2d();
    private Rotation2d rollAdjustmentAngle = new Rotation2d();
    private Rotation2d pitchAdjustmentAngle = new Rotation2d();

    public Rotation2d getYaw() {
        Rotation2d angle = getUnadjustedYaw().minus(yawAdjustmentAngle);
        return angle;
    }

    public Rotation2d getRoll() {
        return getUnadjustedRoll().minus(rollAdjustmentAngle);
    }

    public Rotation2d getPitch() {
        return getUnadjustedPitch().minus(pitchAdjustmentAngle).unaryMinus();
    }

    /**
     * Sets the yaw register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setYaw(double angleDeg) {
        yawAdjustmentAngle = getUnadjustedYaw().minus(Rotation2d.fromDegrees(angleDeg));
    }

    /**
     * Sets the roll register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setRoll(double angleDeg) {
        rollAdjustmentAngle = getUnadjustedRoll().minus(Rotation2d.fromDegrees(angleDeg));
    }

    /**
     * Sets the pitch register to read the specified value.
     *
     * @param angleDeg New yaw in degrees
     */
    public void setPitch(double angleDeg) {
        pitchAdjustmentAngle = getUnadjustedPitch().minus(Rotation2d.fromDegrees(angleDeg));
    }

    public Rotation2d getUnadjustedYaw() {
        return Rotation2d.fromDegrees(mGyro.getAngle());
    }

    public Rotation2d getUnadjustedPitch() {
        return Rotation2d.fromDegrees(mGyro.getYComplementaryAngle());
    }

    public Rotation2d getUnadjustedRoll() {
        return Rotation2d.fromDegrees(mGyro.getXComplementaryAngle());
    }

    public Rotation2d getYawOffset() {
        return yawAdjustmentAngle;
    }
}
