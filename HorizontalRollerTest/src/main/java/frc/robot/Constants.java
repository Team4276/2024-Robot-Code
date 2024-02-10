package frc.robot;

public class Constants {
    public static class OIConstants{
        public static double kJoystickDeadband = 0.05;
    }

    public static class ShooterConstants{
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kS_Top = 0.137;
        public static double kS_Bottom = 0.1275;
        // Math: (V * S / m) / 60 sec / 39.37 in/m * circumference of flywheel 
        public static double kV = 0.0020614125023555073743193198053;
        public static double kA = 0;
    }
}
