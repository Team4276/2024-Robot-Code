package frc.team4276.frc2024;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.util.InterpolatingDouble;
import frc.team254.lib.util.InterpolatingTreeMap;
import frc.team4276.frc2024.Constants.LimelightConstants;
import frc.team4276.frc2024.subsystems.LimeLight.VisionUpdate;

public class RobotState{
    private static RobotState mInstance;
    private Optional<VisionUpdate> mLatestVisionUpdate;
    
    private InterpolatingTreeMap<InterpolatingDouble, Pose2d> odom_to_vehicle_;
    
    private static final int kObservationBufferSize = 50;

    private boolean mHasBeenEnabled = false;

    public static RobotState getInstance(){
        if(mInstance == null){
            mInstance = new RobotState();
        }
        return mInstance;
    }

    private RobotState(){
        reset(0.0, Pose2d.identity());

    }

    public synchronized void reset(double start_time, Pose2d initial_odom_to_vehicle) {
        odom_to_vehicle_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        odom_to_vehicle_.put(new InterpolatingDouble(start_time), initial_odom_to_vehicle);
        // field_to_odom_ = new InterpolatingTreeMap<>(kObservationBufferSize);
        // field_to_odom_.put(new InterpolatingDouble(start_time), getInitialFieldToOdom().getTranslation());
        // vehicle_velocity_predicted_ = Twist2d.identity();
        // vehicle_velocity_measured_ = Twist2d.identity();
        // vehicle_velocity_measured_filtered_ = new MovingAverageTwist2d(25);
        mLatestVisionUpdate = Optional.empty();
        // mDisplayVisionPose = Pose2d.identity();
        // mSetpointPose = Pose2d.identity();
        // mPoseAcceptor = new VisionPoseAcceptor();

        // mField2d = new Field2d();
        // mField2d.setRobotPose(Constants.kWidthField2d, Constants.kHeightField2d, new edu.wpi.first.math.geometry.Rotation2d(0));
        //mField2d.getObject("vision").setPose(Constants.kWidthField2d, Constants.kHeightField2d, new edu.wpi.first.math.geometry.Rotation2d(0));
        //mField2d.getObject("fused").setPose(Constants.kWidthField2d, Constants.kHeightField2d, new edu.wpi.first.math.geometry.Rotation2d(0));
    }

    public void visionUpdate(VisionUpdate update){
        mLatestVisionUpdate = Optional.ofNullable(update);
        if (!mLatestVisionUpdate.isEmpty()){
            Pose2d odomToVehicle = getOdomToVehicle(mLatestVisionUpdate.get().getTimestamp());

            Pose2d camToTag = Pose2d.fromTranslation(mLatestVisionUpdate.get().getCameraToTag());

            Pose2d vehicleToTag = Pose2d.fromTranslation(
                LimelightConstants.kLimeLightRobotOffset.transformBy(camToTag)
                .getTranslation().rotateBy(odomToVehicle.getRotation()));

            
            
                // boolean disabledAndNeverEnabled = DriverStation.isDisabled() && !mHasBeenEnabled;
                // if (/*initial_field_to_odom_.isEmpty() ||*/ disabledAndNeverEnabled) {
                //     // var odom_to_vehicle_translation = disabledAndNeverEnabled ? Translation2d.identity() : getOdomToVehicle(visionTimestamp).getTranslation();
                //     // field_to_odom_.put(new InterpolatingDouble(visionTimestamp), visionFieldToVehicle.getTranslation().translateBy(odom_to_vehicle_translation.inverse()));
                //     // initial_field_to_odom_ = Optional.of(field_to_odom_.lastEntry().getValue());
                //     // mKalmanFilter.setXhat(0, field_to_odom_.lastEntry().getValue().x());
                //     // mKalmanFilter.setXhat(1, field_to_odom_.lastEntry().getValue().y());
                //     // mDisplayVisionPose = visionFieldToVehicle;
    
                // } else if (DriverStation.isEnabled()) { 
                //     var field_to_odom = visionFieldToVehicle.getTranslation().translateBy(odomToVehicle.getTranslation().inverse());
                //     if(DriverStation.isAutonomous()) {
                //         final double kMaxDistanceToAccept = 2.0;
                //         if (field_to_odom.inverse().translateBy(field_to_odom_.lastEntry().getValue()).norm() > kMaxDistanceToAccept) {
                //             System.out.println("Invalid vision update!");
                //             return;
                //         }
                //     }
    
                //     mDisplayVisionPose = visionFieldToVehicle;
                //     try {
                //         mKalmanFilter.correct(VecBuilder.fill(0.0, 0.0), VecBuilder.fill(field_to_odom.getTranslation().x(), field_to_odom.getTranslation().y()));
                //         field_to_odom_.put(new InterpolatingDouble(visionTimestamp), Pose2d.fromTranslation(new Translation2d(mKalmanFilter.getXhat(0), mKalmanFilter.getXhat(1))).getTranslation());
                //     } catch (Exception e) {
                //         DriverStation.reportError("QR Decomposition failed: ", e.getStackTrace());
                //     }
                // } else {
                //     mDisplayVisionPose = null;
                // }



        }

    }

    /**
     * Returns the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized Pose2d getOdomToVehicle(double timestamp) {
        return odom_to_vehicle_.getInterpolated(new InterpolatingDouble(timestamp));
    }

    public synchronized void addOdomToVehicleObservation(double timestamp, Pose2d observation) {
        odom_to_vehicle_.put(new InterpolatingDouble(timestamp), observation);
    }

    public synchronized void setHasBeenEnabled(boolean hasBeenEnabled) {
        mHasBeenEnabled = hasBeenEnabled;
    }


    
}