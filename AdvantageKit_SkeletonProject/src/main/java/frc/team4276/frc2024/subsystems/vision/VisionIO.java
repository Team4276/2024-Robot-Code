package frc.team4276.frc2024.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface VisionIO {

  // TODO: expand for other inputs
  // @AutoLog
  class VisionIOInputs {
    public PhotonPipelineResult result;

    public Pose3d estimatedPose;

    public double timestampSeconds;

    public List<PhotonTrackedTarget> targetsUsed;

    public boolean isValid;
  }

  void updateInputs(VisionIOInputs inputs);
}
