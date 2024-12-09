package frc.team4276.frc2024.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform3d;

import frc.team4276.frc2024.RobotState;
import frc.team4276.lib.util.VirtualSubsystem;

public class Vision extends VirtualSubsystem {
    private final VisionIO[] ios;
    private final VisionIOInputsAutoLogged[] inputs;

    public Vision(VisionIO... ios) {
        this.ios = ios;
        inputs = new VisionIOInputsAutoLogged[ios.length];
        for (int i = 0; i < ios.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }
    }

    //TODO: impl rio logic vision
    @Override
    public void periodic() { //TODO: impl fudge factors 
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera " + i, inputs[i]);

            if (!inputs[i].isConnected || !inputs[i].hasTargets)
                continue;

            double lowest_dist = Double.POSITIVE_INFINITY;
            double total_tag_dist = 0.0;

            for (Transform3d target : inputs[i].bestTargets) {
                total_tag_dist += target.getTranslation().getNorm();
            }

            double avg_dist = total_tag_dist / inputs[i].bestTargets.length;

            double std_dev_multiplier = 1.0;

            double distStDev = std_dev_multiplier
                * (0.1)
                * ((0.01 * Math.pow(lowest_dist, 2.0)) + (0.005 * Math.pow(avg_dist, 2.0)))
                / inputs[i].bestTargets.length;

            distStDev = Math.max(0.02, distStDev);

            RobotState.getInstance().visionUpdate(new RobotState.VisionUpdate(
                inputs[i].timestampSeconds,
                inputs[i].estimatedPose.toPose2d(),
                distStDev));
        }
    }
}
