package frc.team4276.frc2024.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionIOPhoton implements VisionIO {
    private final PhotonCamera mCamera;
    private final PhotonDeviceConstants mConstants;
    private final PhotonPoseEstimator mPoseEstimator;

    public static class PhotonDeviceConstants {
        public String kCameraNameId = "ERROR_ASSIGN_A_NAME";
        public String kCameraName = "ERROR_ASSIGN_A_NAME";
        public Transform3d kRobotToCamera = new Transform3d();
    }

    public VisionIOPhoton(final PhotonDeviceConstants constants) {
        mConstants = constants;

        mCamera = new PhotonCamera(mConstants.kCameraNameId);

        mPoseEstimator = new PhotonPoseEstimator(
                AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                mCamera,
                mConstants.kRobotToCamera);

        mPoseEstimator.setMultiTagFallbackStrategy(
                PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS);
    }

    //TODO: impl rio logic vision
    @Override
    public void updateInputs(VisionIOInputs inputs) { //TODO: impl fudge factors 
        inputs.isConnected = mCamera.isConnected();

        if(!inputs.isConnected){
            return;
        }

        PhotonPipelineResult result = mCamera.getLatestResult();

        Optional<EstimatedRobotPose> pose = mPoseEstimator.update(result);

        if (pose.isEmpty())
            return;

        inputs.estimatedPose = pose.get().estimatedPose;
        inputs.timestampSeconds = result.getTimestampSeconds();
        inputs.bestTargets = result.targets.stream().map(target -> target.getBestCameraToTarget()).toArray(Transform3d[]::new);
        inputs.altTargets = result.targets.stream().map(target -> target.getAlternateCameraToTarget()).toArray(Transform3d[]::new);
        inputs.targetAmbiguities = result.targets.stream().mapToDouble(target -> target.getPoseAmbiguity()).toArray();
    }
}
