package frc.team4276.lib.rev;

public class RevUtil {
    public static class SparkAbsoluteEncoderConfig {
        public boolean kIsInverted = false; // Encoder
        public double kUnitsPerRotation = 1.0; // Overall Rotation
        public double kOffset = Double.NaN; // Set to NaN to not configure
        public int kAvgSamplingDepth = 128;
        public double kPeriodicFrameTime = 0.02;
    }
    
    public static class SparkRelativeEncoderConfig {
        public boolean kIsInverted = false; // Encoder
        public double kUnitsPerRotation = 1.0; // Overall Rotation
        public int kAvgSamplingDepth = 64;
        public int kMeasurementPeriod = 100;
        public double kPeriodicFrameTime = 0.02;
    }
}
