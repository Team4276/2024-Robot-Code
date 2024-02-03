package frc.team4276.frc2024;

import java.util.Optional;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.estimator.UnscentedKalmanFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Translation2d;
import frc.team254.lib.util.InterpolatingDouble;
import frc.team254.lib.util.InterpolatingTreeMap;

import frc.team4276.frc2024.Constants.LimelightConstants;
import frc.team4276.frc2024.Constants.RobotStateConstants;
import frc.team4276.frc2024.Limelight.VisionPoseAcceptor;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.LimeLight.VisionUpdate;

public class RobotState {
    private static RobotState mInstance;
    private Optional<VisionUpdate> mLatestVisionUpdate;
    private UnscentedKalmanFilter<N2, N2, N2> mKalmanFilter;

    private VisionPoseAcceptor mPoseAcceptor;

    private Optional<Translation2d> initial_field_to_odom_ = Optional.empty();
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> odom_to_vehicle_;
    private InterpolatingTreeMap<InterpolatingDouble, Translation2d> field_to_odom_;

    private static final int kObservationBufferSize = 50;

    private boolean mHasBeenEnabled = false;

    public static RobotState getInstance() {
        if (mInstance == null) {
            mInstance = new RobotState();
        }
        return mInstance;
    }

    private RobotState() {
        reset(0.0, Pose2d.identity());

    }

    public synchronized void reset(double start_time, Pose2d initial_odom_to_vehicle) {
        odom_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        odom_to_vehicle_.put(new InterpolatingDouble(start_time), initial_odom_to_vehicle);
        field_to_odom_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        field_to_odom_.put(new InterpolatingDouble(start_time), getInitialFieldToOdom().getTranslation());
        // vehicle_velocity_predicted_ = Twist2d.identity();
        // vehicle_velocity_measured_ = Twist2d.identity();
        // vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
        mLatestVisionUpdate = Optional.empty();
        // mDisplayVisionPose = Pose2d.identity();
        // mSetpointPose = Pose2d.identity();
        mPoseAcceptor = new VisionPoseAcceptor();

        // mField2d = new Field2d();
        // mField2d.setRobotPose(Constants.kWidthField2d, Constants.kHeightField2d, new
        // edu.wpi.first.math.geometry.Rotation2d(0));
        // mField2d.getObject("vision").setPose(Constants.kWidthField2d,
        // Constants.kHeightField2d, new edu.wpi.first.math.geometry.Rotation2d(0));
        // mField2d.getObject("fused").setPose(Constants.kWidthField2d,
        // Constants.kHeightField2d, new edu.wpi.first.math.geometry.Rotation2d(0));
    }

    public synchronized void resetKalmanFilters() {
        mKalmanFilter =
        new UnscentedKalmanFilter<>(
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

    public void visionUpdate(VisionUpdate update) {
        mLatestVisionUpdate = Optional.ofNullable(update);
        if (!mLatestVisionUpdate.isEmpty()) {
            double visionTimestamp = mLatestVisionUpdate.get().getTimestamp();

            Pose2d odomToVehicle = getOdomToVehicle(visionTimestamp);

            Pose2d camToTag = Pose2d.fromTranslation(mLatestVisionUpdate.get().getCameraToTag());

            Pose2d vehicleToTag = Pose2d.fromTranslation(
                    LimelightConstants.kLimeLightRobotOffset.transformBy(camToTag)
                            .getTranslation().rotateBy(odomToVehicle.getRotation()));

            Pose2d visionFieldToVehicle = mLatestVisionUpdate.get().getTagInField().transformBy(vehicleToTag.inverse());

            if (!mPoseAcceptor.shouldAcceptVision(vehicleToTag, DriveSubsystem.getInstance().getMeasSpeeds())){
                return;
            }
            boolean disabledAndNeverEnabled = DriverStation.isDisabled() && !mHasBeenEnabled;
            if (initial_field_to_odom_.isEmpty() || disabledAndNeverEnabled) {
                var odom_to_vehicle_translation = disabledAndNeverEnabled ?
                Translation2d.identity() :
                getOdomToVehicle(visionTimestamp).getTranslation();
                field_to_odom_.put(new InterpolatingDouble(visionTimestamp),
                visionFieldToVehicle.getTranslation().translateBy(odom_to_vehicle_translation.inverse()));
                initial_field_to_odom_ = Optional.of(field_to_odom_.lastEntry().getValue());
                mKalmanFilter.setXhat(0, field_to_odom_.lastEntry().getValue().x());
                mKalmanFilter.setXhat(1, field_to_odom_.lastEntry().getValue().y());
                // mDisplayVisionPose = visionFieldToVehicle;

            } else if (DriverStation.isEnabled()) {
                var field_to_odom = visionFieldToVehicle.getTranslation()
                        .translateBy(odomToVehicle.getTranslation().inverse());
                if (DriverStation.isAutonomous()) {
                    final double kMaxDistanceToAccept = 5.0;
                    if (field_to_odom.inverse().translateBy(field_to_odom_.lastEntry().getValue())
                            .norm() > kMaxDistanceToAccept) {
                        System.out.println("Invalid vision update!");
                        return;
                    }
                }

                // mDisplayVisionPose = visionFieldToVehicle;
                try {
                    mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0),
                            VecBuilder.fill(field_to_odom.getTranslation().x(), field_to_odom.getTranslation().y()));

                    field_to_odom_.put(new InterpolatingDouble(visionTimestamp),
                            Pose2d.fromTranslation(
                                    new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1)))
                                    .getTranslation());
                } catch (Exception e) {
                    throw e;
                    //DriverStation.reportError("QR Decomposition failed: ", e.getStackTrace());
                }
            } else {
                // mDisplayVisionPose = null;
            }

        }

    }

    /**
     * Return Initial Vision Offset for Pure Odometry Visualization Purposes
     * @return
     */
    public synchronized Pose2d getInitialFieldToOdom() {
        if (initial_field_to_odom_.isEmpty()) return Pose2d.identity();
        return Pose2d.fromTranslation(initial_field_to_odom_.get());
    }

    public synchronized Translation2d getFieldToOdom(double timestamp) {
        if (initial_field_to_odom_.isEmpty()) return Translation2d.identity();
        return initial_field_to_odom_.get().inverse().translateBy(field_to_odom_.getInterpolated(new InterpolatingDouble(timestamp)));
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

    public synchronized Pose2d getCurrentFieldToVehicle(){
        return getFieldToVehicleAbsolute(Timer.getFPGATimestamp());
    }

    public synchronized edu.wpi.first.math.geometry.Pose2d getWPICurrentFieldToVehicle(){
        return Pose2d.toWPI(getCurrentFieldToVehicle());
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

            //TODO: look into throwables and error catching
        }
        addOdomToVehicleObservation(timestamp, odom_to_robot);
    }

    public synchronized void reset() {
        reset(Timer.getFPGATimestamp(), Pose2d.identity());
    }


}