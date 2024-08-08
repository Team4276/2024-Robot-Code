package frc.team4276.frc2024;

import java.util.NoSuchElementException;
import java.util.Optional;

import javax.swing.text.html.Option;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.ExtendedKalmanFilter;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.field.Field;
import frc.team4276.frc2024.subsystems.vision.VisionPoseAcceptor;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.util.InterpolatingDouble;
import frc.team254.lib.util.InterpolatingTreeMap;

//TODO: refactor variable names
public class RobotState {
    private Translation2d mEstimatedPose = Translation2d.identity();

    private ExtendedKalmanFilter<N2, N2, N2> mKalmanFilter;

    private VisionPoseAcceptor mPoseAcceptor;

    private static final double kObservationBufferTime = 1.0;

    private final TimeInterpolatableBuffer<edu.wpi.first.math.geometry.Pose2d> mOdomPoseBuffer;

    private boolean mHasBeenEnabled = false;

    private Field.POIs mPOIs;

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
        mOdomPoseBuffer.clear();
        mOdomPoseBuffer.addSample(start_time, initial_pose.toWPI());
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }

    public synchronized void resetKalmanFilters() {
        mKalmanFilter = new ExtendedKalmanFilter<N2, N2, N2>(
                Nat.N2(), // Dimensions of output (x, y)
                Nat.N2(), // Dimensions of predicted error shift (dx, dy) (always 0)
                Nat.N2(), // Dimensions of vision (x, y)
                (x, u) -> u, // The derivative of the output is predicted shift (always 0)
                (x, u) -> x, // The output is position (x, y)
                Constants.RobotStateConstants.kStateStdDevs, // Standard deviation of position (uncertainty propagation with no vision)
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

    public synchronized void setHasBeenEnabled(boolean hasBeenEnabled) {
        mHasBeenEnabled = hasBeenEnabled;
    }

    public synchronized void addOdomObservations(double timestamp, Pose2d odom_to_robot) {
        mKalmanFilter.predict(VecBuilder.fill(0.0, 0.0), Constants.kLooperDt);

        mOdomPoseBuffer.addSample(timestamp, odom_to_robot.toWPI());
    }

    public static class VisionUpdate {
        public final double timestamp;
        public final Translation2d fieldToVis;

        public VisionUpdate(double timestamp, Translation2d fieldToVis) {
            this.timestamp = timestamp;
            this.fieldToVis = fieldToVis;
        }
    }

    public synchronized void visionUpdate(VisionUpdate update) {
        double visionTimestamp = update.timestamp;

        try {
            if (mOdomPoseBuffer.getInternalBuffer().lastKey() - kObservationBufferTime > visionTimestamp)
                return;

        } catch (NoSuchElementException e) {
            return;
        }

        Optional<edu.wpi.first.math.geometry.Pose2d> sample = mOdomPoseBuffer.getSample(visionTimestamp);

        if (sample.isEmpty())
            return;

        Transform2d sampleToOdom = new Transform2d(sample.get(), mOdomPoseBuffer.getInternalBuffer().lastEntry().getValue());

        //TODO: fix logic
        if (!mPoseAcceptor.shouldAcceptVision(DriveSubsystem.getInstance().getMeasSpeeds()))
            return;

        boolean disabledAndNeverEnabled = DriverStation.isDisabled() && !mHasBeenEnabled;
        // if (initial_field_to_odom_.isEmpty() || disabledAndNeverEnabled) {
        //     var odom_to_vehicle_translation = disabledAndNeverEnabled ? Translation2d.identity()
        //             : getOdomToVehicle(visionTimestamp).getTranslation();
        //     field_to_odom_.put(new InterpolatingDouble(visionTimestamp),
        //             update.fieldToVis.translateBy(odom_to_vehicle_translation.inverse()));
        //     initial_field_to_odom_ = Optional.of(field_to_odom_.lastEntry().getValue());

        //     return;

        // }

        mEstimatedPose = update.fieldToVis.translateBy(Translation2d.fromWPI(sampleToOdom.getTranslation()));

        try {
            mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0), 
                VecBuilder.fill(mEstimatedPose.x(), mEstimatedPose.y()), null);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public synchronized Pose2d getLatestFieldToVehicle() {
        return new Pose2d(mEstimatedPose, Rotation2d.fromWPI(mOdomPoseBuffer.getInternalBuffer().lastEntry().getValue().getRotation()));
    }

    public synchronized edu.wpi.first.math.geometry.Pose2d getWPILatestFieldToVehicle() {
        return getLatestFieldToVehicle().toWPI();
    }
}