package frc.team4276.frc2024;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N2;
import frc.team4276.frc2024.field.Field;
import frc.team4276.frc2024.shooting.RegressionMaps;
import frc.team4276.frc2024.subsystems.drive.DriveConstants;
import frc.team4276.lib.MovingAverage;

public class RobotState {
    private final SwerveDriveOdometry mOdometry;

    private Translation2d mEstimatedPose = new Translation2d();
    private MovingAverage mEstimatedVisionHeading = new MovingAverage(100);

    private ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter;
    private boolean mHasUpdated = false;

    private static final double kObservationBufferTime = 1.0;

    private final TimeInterpolatableBuffer<Pose2d> mOdomPoseBuffer;

    private Field.POIs mPOIs = Field.Red.kPOIs;

    public record AimingParameters(
            Rotation2d driveHeading,
            double fourbarSetpoint,
            double flywheelRpm,
            double distance) {

        public Rotation2d getDriveHeading() {
            return driveHeading;
        }

        public double getFourbarSetpoint() {
            return fourbarSetpoint;
        }

        public double getFlywheelRpm() {
            return flywheelRpm;
        }

        public double getDistance() {
            return distance;
        }
    }

    private AimingParameters latestSpeakerParams;
    private AimingParameters latestFerryParams;

    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }
        return mInstance;
    }

    private RobotState() {
        mOdometry = new SwerveDriveOdometry(DriveConstants.kKinematics, new Rotation2d(), new SwerveModulePosition[4]);
        mOdomPoseBuffer = TimeInterpolatableBuffer.createBuffer(kObservationBufferTime);

        reset(0.0, new Pose2d());
    }

    public synchronized void reset(double start_time, Pose2d initial_pose) {
        mOdomPoseBuffer.addSample(start_time, initial_pose);
        mEstimatedPose = initial_pose.getTranslation();
    }

    public synchronized void resetKalmanFilters() {
        mKalmanFilter = new ExtendedKalmanFilter<N2, N2, N2>(
                Nat.N2(), // Dimensions of output (x, y)
                Nat.N2(), // Dimensions of predicted error shift (dx, dy) (always 0)
                Nat.N2(), // Dimensions of vision (x, y)
                (x, u) -> u, // The derivative of the output is predicted shift (always 0)
                (x, u) -> x, // The output is position (x, y)
                Constants.RobotStateConstants.kStateStdDevs, // Standard deviation of position (uncertainty propagation
                // with no vision)
                Constants.RobotStateConstants.kLocalMeasurementStdDevs, // Standard deviation of vision measurements
                Constants.kLooperDt);
    }

    public synchronized Field.POIs getPOIs() {
        return mPOIs;
    }

    public synchronized void setBlue() {
        mPOIs = Field.Blue.kPOIs;
    }

    public synchronized void setRed() {
        mPOIs = Field.Red.kPOIs;
    }

    public synchronized void visionHeadingUpdate(double heading_rad) {
        mEstimatedVisionHeading.addNumber(heading_rad);
    }

    public synchronized void addOdomObservations(double timestamp, SwerveDriveWheelPositions positions, Rotation2d heading) {
        latestSpeakerParams = null;
        latestFerryParams = null;

        mOdometry.update(heading, positions);

        mKalmanFilter.predict(VecBuilder.fill(0.0, 0.0), Constants.kLooperDt);
        
        mEstimatedPose = mEstimatedPose.plus(mOdometry.getPoseMeters().getTranslation().minus(
            mOdomPoseBuffer.getInternalBuffer().lastEntry().getValue().getTranslation()));

        mOdomPoseBuffer.addSample(timestamp, mOdometry.getPoseMeters());
    }

    public static class VisionUpdate {
        public final double timestamp;
        public final Translation2d fieldToVis;
        public final double distStDev;

        public VisionUpdate(double timestamp, Translation2d fieldToVis, double distStDev) {
            this.timestamp = timestamp;
            this.fieldToVis = fieldToVis;
            this.distStDev = distStDev;
        }
    }


    public synchronized void visionUpdate(VisionUpdate update) {
        double visionTimestamp = update.timestamp;

        if (mOdomPoseBuffer.getInternalBuffer().lastKey() - kObservationBufferTime > visionTimestamp)
            return;

        mEstimatedPose = update.fieldToVis.plus(mOdomPoseBuffer.getInternalBuffer().lastEntry().getValue()
                        .getTranslation().minus(mOdomPoseBuffer.getSample(visionTimestamp).get().getTranslation()));

        if (!mHasUpdated) {
            mKalmanFilter.setXhat(0, mEstimatedPose.getX());
            mKalmanFilter.setXhat(1, mEstimatedPose.getY());

            mHasUpdated = true;
            return;
        }

        try {
            mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0),
                    VecBuilder.fill(mEstimatedPose.getX(), mEstimatedPose.getY()),
                    StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), VecBuilder.fill(update.distStDev, update.distStDev)));
            mEstimatedPose = new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1));

        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    public synchronized Pose2d getLatestFieldToVehicle() {
        return new Pose2d(mEstimatedPose, mOdomPoseBuffer.getInternalBuffer().lastEntry().getValue().getRotation());
    }

    public AimingParameters getSpeakerAimingParameters() {
        if (latestSpeakerParams != null) {
            return latestSpeakerParams; // return cached params if no new updates
        }

        Translation2d robot_to_target = getPOIs().kSpeakerCenter.minus(getLatestFieldToVehicle().getTranslation());

        double distance = robot_to_target.getNorm();
        double armAngle = RegressionMaps.kSpeakerFourbarAngles.get(distance);
        double flywheel_speeds = RegressionMaps.kSpeakerFlywheelRPMs.get(distance);
        
        latestSpeakerParams = new AimingParameters(robot_to_target.getAngle(), armAngle, flywheel_speeds, distance);

        return latestSpeakerParams;
    }

    public AimingParameters getFerryAimingParameters() {
        if (latestFerryParams != null) {
            return latestFerryParams; // return cached params if no new updates
        }

        Translation2d robot_to_target = getPOIs().kSpeakerCenter.minus(getLatestFieldToVehicle().getTranslation());

        double distance = robot_to_target.getNorm();
        double armAngle = RegressionMaps.kFerryFourbarAngles.get(distance);
        double flywheel_speeds = RegressionMaps.kFerryFlywheelRPMs.get(distance);

        latestFerryParams = new AimingParameters(robot_to_target.getAngle(), armAngle, flywheel_speeds, distance);

        return latestFerryParams;
    }

    /** Use on auton init */
    public synchronized double getHeadingFromVision() {
        if (mEstimatedVisionHeading.getSize() == 0) {
            return getLatestFieldToVehicle().getRotation().getRadians();
        }
        return mEstimatedVisionHeading.getAverage();
    }
}
