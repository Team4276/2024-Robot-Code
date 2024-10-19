package frc.team4276.frc2024;

import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;

import frc.team4276.frc2024.field.Field;
import frc.team4276.frc2024.subsystems.vision.VisionPoseAcceptor;
import frc.team4276.frc2024.subsystems.DriveSubsystem;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.util.MovingAverage;

public class RobotState {
    private Translation2d mEstimatedPose = Translation2d.identity();
    private MovingAverage mEstimatedVisionHeading = new MovingAverage(100);

    private ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter;
    private boolean mHasUpdated = false;

    private VisionPoseAcceptor mPoseAcceptor;

    private static final double kObservationBufferTime = 1.0;

    private final TimeInterpolatableBuffer<edu.wpi.first.math.geometry.Pose2d> mOdomPoseBuffer;

    private Field.POIs mPOIs = Field.Red.kPOIs;

    private static RobotState mInstance;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }
        return mInstance;
    }

    private RobotState() {
        mOdomPoseBuffer = TimeInterpolatableBuffer.createBuffer(kObservationBufferTime);
        mPoseAcceptor = new VisionPoseAcceptor();

        reset(0.0, Pose2d.identity());

    }

    public synchronized void reset(double start_time, Pose2d initial_pose) {
        mOdomPoseBuffer.addSample(start_time, initial_pose.toWPI());
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

    public synchronized void addOdomObservations(double timestamp, Pose2d odom_to_robot) {
        mKalmanFilter.predict(VecBuilder.fill(0.0, 0.0), Constants.kLooperDt);

        Translation2d dx = odom_to_robot.getTranslation().translateBy(
            Translation2d.fromWPI(mOdomPoseBuffer.getInternalBuffer().lastEntry().getValue().getTranslation()).inverse());

        SmartDashboard.putNumber("Debug/Vision Delta X", dx.x());
        SmartDashboard.putNumber("Debug/Vision Delta Y", dx.y());

        mEstimatedPose.translateBy(dx);

        mOdomPoseBuffer.addSample(timestamp, odom_to_robot.toWPI());
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

        Pose2d sample = Pose2d.fromWPI(mOdomPoseBuffer.getSample(visionTimestamp).get());

        if (!mPoseAcceptor.shouldAcceptVision(DriveSubsystem.getInstance().getMeasSpeeds()))
            return;

        mEstimatedPose = update.fieldToVis.translateBy(Translation2d.fromWPI(mOdomPoseBuffer.getInternalBuffer().lastEntry().getValue()
                .getTranslation()).translateBy(sample.getTranslation().inverse()));

        if(!mHasUpdated) {
            mKalmanFilter.setXhat(0, mEstimatedPose.x());
            mKalmanFilter.setXhat(1, mEstimatedPose.y());

            mHasUpdated = true;
            return;
        }

        try {
            mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0),
                    VecBuilder.fill(mEstimatedPose.getTranslation().x(), mEstimatedPose.getTranslation().y()),
                    StateSpaceUtil.makeCovarianceMatrix(Nat.N2(), VecBuilder.fill(update.distStDev, update.distStDev)));
            mEstimatedPose.translateBy(new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1)));

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    // Use on enabled init
    public synchronized double getHeadingFromVision() {
        if(mEstimatedVisionHeading.getSize() == 0) {
            return getLatestFieldToVehicle().getRotation().getRadians();
        }
        return mEstimatedVisionHeading.getAverage();
    }

    public synchronized Pose2d getLatestFieldToVehicle() {
        return new Pose2d(mEstimatedPose,
                Rotation2d.fromWPI(mOdomPoseBuffer.getInternalBuffer().lastEntry().getValue().getRotation()));
    }

    public synchronized edu.wpi.first.math.geometry.Pose2d getWPILatestFieldToVehicle() {
        edu.wpi.first.math.geometry.Pose2d p = getLatestFieldToVehicle().toWPI();
        SmartDashboard.putNumber("Debug/WPI Field to Vehicle X", p.getX());
        SmartDashboard.putNumber("Debug/WPI Field to Vehicle Y", p.getY());
        SmartDashboard.putNumber("Debug/WPI Field to Vehicle Rot", p.getRotation().getDegrees());
        return p;
    }
}