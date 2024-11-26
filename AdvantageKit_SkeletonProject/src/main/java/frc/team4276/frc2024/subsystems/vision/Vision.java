package frc.team4276.frc2024.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2024.RobotState;

public class Vision extends SubsystemBase {
    private final VisionIO[] ios;
    private final VisionIOInputsAutoLogged[] inputs;

    public Vision(VisionIO... ios) {
        this.ios = ios;
        inputs = new VisionIOInputsAutoLogged[ios.length];
        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    @Override
    public void periodic() { //TODO: add vision constants
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera " + i, inputs[i]);
        }

        for (VisionIOInputsAutoLogged input : inputs) {
            if (!input.isConnected || !input.hasTargets)
                continue;

            double lowest_dist = Double.POSITIVE_INFINITY;
            double total_tag_dist = 0.0;

            for (Transform3d target : input.bestTargets) {
                total_tag_dist += target.getTranslation().getNorm();
            }

            double avg_dist = total_tag_dist / input.bestTargets.length;

            double std_dev_multiplier = 1.0;

            double distStDev = std_dev_multiplier
                * (0.1)
                * ((0.01 * Math.pow(lowest_dist, 2.0)) + (0.005 * Math.pow(avg_dist, 2.0)))
                / input.bestTargets.length;

            distStDev = Math.max(0.02, distStDev);

            RobotState.getInstance().visionUpdate(new RobotState.VisionUpdate(
                input.timestampSeconds,
                input.estimatedPose.toPose2d(),
                distStDev));
        }
    }
}
