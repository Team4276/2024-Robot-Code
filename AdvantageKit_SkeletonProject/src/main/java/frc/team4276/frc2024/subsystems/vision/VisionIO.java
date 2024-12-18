package frc.team4276.frc2024.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public boolean isConnected;
        public boolean hasTargets;

        public Pose3d estimatedPose;
        public double timestampSeconds;
        public Transform3d[] bestTargets;
        public Transform3d[] altTargets;
        public double[] targetAmbiguities;
    }

    default void updateInputs(VisionIOInputs inputs){}
}
