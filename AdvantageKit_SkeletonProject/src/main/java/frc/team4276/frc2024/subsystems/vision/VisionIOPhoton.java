package frc.team4276.frc2024.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

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

    mPoseEstimator =
        new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            mCamera,
            mConstants.kRobotToCamera);

    mPoseEstimator.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.isValid = true;

    PhotonPipelineResult result = mCamera.getLatestResult();

    if (!result.hasTargets()) {
      inputs.isValid = false;
      return;
    }

    Optional<EstimatedRobotPose> pose = mPoseEstimator.update(result);

    if (pose.isEmpty()) {
      inputs.isValid = false;
      return;
    }

    inputs.result = result;
    inputs.estimatedPose = pose.get().estimatedPose;
    inputs.targetsUsed = pose.get().targetsUsed;
    inputs.timestampSeconds = pose.get().timestampSeconds;
  }
}
