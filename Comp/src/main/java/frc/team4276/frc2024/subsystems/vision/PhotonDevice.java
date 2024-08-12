package frc.team4276.frc2024.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.field.Field;
import frc.team4276.lib.drivers.Subsystem;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

import frc.team254.lib.geometry.Translation2d;

public class PhotonDevice extends Subsystem {
    private PhotonCamera mCamera;
    private PhotonDeviceConstants mConstants;

    private PhotonPoseEstimator mPoseEstimator;

    private Pose3d mLatestUpdate;

    public static class PhotonDeviceConstants {
        public String kCameraNameId = "ERROR_ASSIGN_A_NAME";
        public String kCameraName = "ERROR_ASSIGN_A_NAME";
        public Transform3d kRobotToCamera = new Transform3d();
    }

    public PhotonDevice(PhotonDeviceConstants constants) {
        mConstants = constants;

        mCamera = new PhotonCamera(mConstants.kCameraNameId);
        
        mPoseEstimator = new PhotonPoseEstimator(AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mCamera, mConstants.kRobotToCamera);

        mPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS);
    }

    public String getNameId() {
        return mConstants.kCameraNameId;
    }

    public String getName() {
        return mConstants.kCameraName;
    }
  
    public boolean getConnected(){
        return mCamera.isConnected();
    }

    public Pose3d getLatestUpdate(){
        return mLatestUpdate;
    }
    //TODO: check vision readings vs gyro; check height reading (citrus ignore)

    @Override
    public void readPeriodicInputs() {
        PhotonPipelineResult result = mCamera.getLatestResult();

        if(!result.hasTargets()) 
            return;

        Optional<EstimatedRobotPose> estimatedRobotPose = mPoseEstimator.update(result);

        if(estimatedRobotPose.isPresent()) {
            //TODO: calibrate std dev scaling
            double total_tag_dist = 0.0;
            double lowest_dist = Double.POSITIVE_INFINITY;

            for(PhotonTrackedTarget target : estimatedRobotPose.get().targetsUsed) {
                double dist = estimatedRobotPose.get().estimatedPose.getTranslation().toTranslation2d().getDistance(Field.kAprilTagMap.get(target.getFiducialId()).getTagInField().toWPI().getTranslation()) ;
				total_tag_dist += dist;
				lowest_dist = Math.min(dist, lowest_dist);
            }

            double avg_dist = total_tag_dist / estimatedRobotPose.get().targetsUsed.size();

            double std_dev_multiplier = 1.0;

            double distStDev = std_dev_multiplier
					* (0.1)
					* ((0.01 * Math.pow(lowest_dist, 2.0)) + (0.005 * Math.pow(avg_dist, 2.0)))
					/ estimatedRobotPose.get().targetsUsed.size();
			distStDev = Math.max(0.02, distStDev);

            RobotState.getInstance().visionUpdate(new RobotState.VisionUpdate(estimatedRobotPose.get().timestampSeconds, 
                new Translation2d(estimatedRobotPose.get().estimatedPose.getX(), estimatedRobotPose.get().estimatedPose.getY()),
                distStDev));

            RobotState.getInstance().visionHeadingUpdate(estimatedRobotPose.get().estimatedPose.getRotation().getZ());

            mLatestUpdate = estimatedRobotPose.get().estimatedPose;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    @Override
    public void writePeriodicOutputs() {
    }


}
