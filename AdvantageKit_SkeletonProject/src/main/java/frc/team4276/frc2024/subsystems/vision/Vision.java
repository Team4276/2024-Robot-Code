package frc.team4276.frc2024.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.field.Field;
import frc.team4276.frc2024.subsystems.vision.VisionIO.VisionIOInputs;
import org.photonvision.targeting.PhotonTrackedTarget;

// TODO: add logging
public class Vision extends SubsystemBase {
    private final VisionIO[] io;
    private final VisionIOInputs[] inputs;

    public Vision(VisionIO... io) {
        this.io = io;
        inputs = new VisionIOInputs[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputs();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
        }

        for (VisionIOInputs input : inputs) {
            if (input.isValid) {
                double total_tag_dist = 0.0;
                double lowest_dist = Double.POSITIVE_INFINITY;

                for (PhotonTrackedTarget target : input.targetsUsed) {
                    double dist = input.estimatedPose
                            .getTranslation()
                            .toTranslation2d()
                            .getDistance(
                                    Field.kAprilTagMap
                                            .get(target.getFiducialId())
                                            .getTagInField()
                                            .getTranslation());
                    total_tag_dist += dist;
                    lowest_dist = Math.min(dist, lowest_dist);
                }

                double avg_dist = total_tag_dist / input.targetsUsed.size();

                double std_dev_multiplier = 1.0;

                double distStDev = std_dev_multiplier
                        * (0.1)
                        * ((0.01 * Math.pow(lowest_dist, 2.0)) + (0.005 * Math.pow(avg_dist, 2.0)))
                        / input.targetsUsed.size();
                distStDev = Math.max(0.02, distStDev);

                RobotState.getInstance()
                        .visionUpdate(
                                new RobotState.VisionUpdate(
                                        input.timestampSeconds,
                                        new Translation2d(input.estimatedPose.getX(), input.estimatedPose.getY()),
                                        distStDev));

                RobotState.getInstance().visionHeadingUpdate(input.estimatedPose.getRotation().getZ());
            }
        }
    }
}
