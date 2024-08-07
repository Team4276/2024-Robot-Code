package frc.team4276.frc2024;

import java.util.Optional;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc.team4276.frc2024.Constants.RobotStateConstants;
import frc.team4276.frc2024.field.Field;
import frc.team4276.frc2024.subsystems.vision.VisionPoseAcceptor;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.util.InterpolatingDouble;
import frc.team254.lib.util.InterpolatingTreeMap;

//TODO: refactor variable names
public class OldRobotState {
    private static OldRobotState mInstance;

    private Optional<VisionUpdate> mLatestVisionUpdate;
    private UnscentedKalmanFilter<N2, N2, N2> mKalmanFilter;

    private boolean mUseKalman = false;

    private VisionPoseAcceptor mPoseAcceptor;

    private Optional<Translation2d> initial_field_to_odom_ = Optional.empty();
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> odom_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Translation2d> field_to_odom_;

    private static final int kObservationBufferSize = 50;

    private boolean mHasBeenEnabled = false;

    private Field.POIs mPOIs;

    public static OldRobotState getInstance() {
        if (mInstance == null) {
            mInstance = new OldRobotState();
        }
        return mInstance;
    }

    private OldRobotState() {
        reset(0.0, Pose2d.identity());

    }

    public synchronized void reset(double start_time, Pose2d initial_odom_to_vehicle) {
        odom_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        odom_to_vehicle_.put(new InterpolatingDouble(start_time), initial_odom_to_vehicle);
        field_to_odom_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_odom_.put(new InterpolatingDouble(start_time), getInitialFieldToOdom().getTranslation());
        mLatestVisionUpdate = Optional.empty();
        mPoseAcceptor = new VisionPoseAcceptor();
    }

    public static class VisionUpdate {
        private double timestamp;
        private Translation2d fieldToRobot;

        public VisionUpdate(double timestamp, Translation2d fieldToRobot) {
            this.timestamp = timestamp;
            this.fieldToRobot = fieldToRobot;
        }

        public double getTimestamp() {
            return timestamp;
        }

        public Translation2d getCameraToTag() {
            return fieldToRobot;
        }
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

    public synchronized void setUseKalman(boolean useKalman) {
        mUseKalman = useKalman;
    }

    //TODO: calc constants
    public synchronized void resetKalmanFilters() {
        mKalmanFilter = new UnscentedKalmanFilter<>(
                Nat.N2(),
                Nat.N2(),
                (x, u) -> VecBuilder.fill(0.0, 0.0),
                (x, u) -> x,
                RobotStateConstants.kStateStdDevs,
                RobotStateConstants.kLocalMeasurementStdDevs, Constants.kLooperDt);

    }

    public synchronized boolean getHasBeenEnabled() {
        return mHasBeenEnabled;
    }

    public synchronized void setHasBeenEnabled(boolean hasBeenEnabled) {
        mHasBeenEnabled = hasBeenEnabled;
    }

    public synchronized void visionUpdate(VisionUpdate update) {
        mLatestVisionUpdate = Optional.ofNullable(update);
        if (mLatestVisionUpdate.isEmpty())
            return;

        double visionTimestamp = mLatestVisionUpdate.get().getTimestamp();

        Pose2d odomToVehicle = getOdomToVehicle(visionTimestamp);

        Translation2d fieldToRobot = mLatestVisionUpdate.get().fieldToRobot;

        if (!mPoseAcceptor.shouldAcceptVision(DriveSubsystem.getInstance().getMeasSpeeds()))
            return;
        

        boolean disabledAndNeverEnabled = DriverStation.isDisabled() && !mHasBeenEnabled;
        if (initial_field_to_odom_.isEmpty() || disabledAndNeverEnabled) {
            var odom_to_vehicle_translation = disabledAndNeverEnabled ? Translation2d.identity()
                    : getOdomToVehicle(visionTimestamp).getTranslation();
            field_to_odom_.put(new InterpolatingDouble(visionTimestamp),
                    fieldToRobot.getTranslation().translateBy(odom_to_vehicle_translation.inverse()));
            initial_field_to_odom_ = Optional.of(field_to_odom_.lastEntry().getValue());
            mKalmanFilter.setXhat(0, field_to_odom_.lastEntry().getValue().x());
            mKalmanFilter.setXhat(1, field_to_odom_.lastEntry().getValue().y());

        } else if (DriverStation.isEnabled()) {
            var field_to_odom = fieldToRobot.getTranslation()
                    .translateBy(odomToVehicle.getTranslation().inverse());

            Translation2d translation;

            if(mUseKalman) {
                try {
                    mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0),
                            VecBuilder.fill(field_to_odom.getTranslation().x(), field_to_odom.getTranslation().y()));
                    
                    translation = new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1));

                } catch (Exception e) {
                    translation = field_to_odom;

                    System.out.println(e.getMessage());

                }

            } else {
                translation = field_to_odom;
                
            }         
            
            field_to_odom_.put(new InterpolatingDouble(visionTimestamp), translation);
        }

    }

    /**
     * Return Initial Vision Offset for Pure Odometry Visualization Purposes
     * 
     * @return
     */
    public synchronized Pose2d getInitialFieldToOdom() {
        if (initial_field_to_odom_.isEmpty())
            return Pose2d.identity();
        return Pose2d.fromTranslation(initial_field_to_odom_.get());
    }

    public synchronized Translation2d getFieldToOdom(double timestamp) {
        if (initial_field_to_odom_.isEmpty())
            return Translation2d.identity();
        return initial_field_to_odom_.get().inverse()
                .translateBy(field_to_odom_.getInterpolated(new InterpolatingDouble(timestamp)));
    }

    public synchronized Translation2d getAbsoluteFieldToOdom(double timestamp) {
        return field_to_odom_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized Translation2d getLatestFieldToOdom() {
        return getFieldToOdom(field_to_odom_.lastKey().value);
    }

    public synchronized Pose2d getFieldToVehicle(double timestamp) {
        Pose2d odomToVehicle = getOdomToVehicle(timestamp);

        Translation2d fieldToOdom = getFieldToOdom(timestamp);
        return new Pose2d(fieldToOdom.translateBy(odomToVehicle.getTranslation()), odomToVehicle.getRotation());

    }

    public synchronized Pose2d getFieldToVehicleAbsolute(double timestamp) {
        var field_to_odom = initial_field_to_odom_.orElse(Translation2d.identity());
        return Pose2d.fromTranslation(field_to_odom).transformBy(getFieldToVehicle(timestamp));
    }

    public synchronized Pose2d getCurrentFieldToVehicle() {
        return getFieldToVehicleAbsolute(Timer.getFPGATimestamp());
    }

    public synchronized edu.wpi.first.math.geometry.Pose2d getWPICurrentFieldToVehicle() {
        return getCurrentFieldToVehicle().toWPI();
    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly
     * interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getOdomToVehicle(double timestamp) {
        return odom_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized void addOdomToVehicleObservation(double timestamp, Pose2d observation) {
        odom_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void addOdomObservations(double timestamp, Pose2d odom_to_robot) {
        try {
            mKalmanFilter.predict(VecBuilder.fill(0.0, 0.0), Constants.kLooperDt);
        } catch (Exception e) {
            throw e;

        }
        addOdomToVehicleObservation(timestamp, odom_to_robot);
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }
}