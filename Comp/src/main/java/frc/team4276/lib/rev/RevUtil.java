package frc.team4276.lib.rev;

public class RevUtil {
    public static class SparkAbsoluteEncoderConfig {
        public boolean kIsInverted = false; // Encoder
        public double kUnitsPerRotation = 1.0; // Overall Rotation
        public double kOffset = 0.0; // Set in hardware client
        public int kAvgSamplingDepth = 128;
    }
}
