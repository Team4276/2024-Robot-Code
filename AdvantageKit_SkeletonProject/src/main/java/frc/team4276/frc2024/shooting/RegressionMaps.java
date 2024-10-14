package frc.team4276.frc2024.shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class RegressionMaps {
    public static InterpolatingDoubleTreeMap kSpeakerFourbarAngles = new InterpolatingDoubleTreeMap();

    static {
        kSpeakerFourbarAngles.put(1.0, 135.0);
        kSpeakerFourbarAngles.put(1.5, 130.0);
        kSpeakerFourbarAngles.put(2.0, 127.0);
        kSpeakerFourbarAngles.put(2.5, 124.0);
        kSpeakerFourbarAngles.put(3.0, 120.0);
        kSpeakerFourbarAngles.put(3.5, 115.0);
        kSpeakerFourbarAngles.put(4.0, 110.0);
        kSpeakerFourbarAngles.put(4.5, 110.0);
        kSpeakerFourbarAngles.put(5.0, 110.0);
        kSpeakerFourbarAngles.put(5.5, 105.0);
        kSpeakerFourbarAngles.put(6.0, 100.0);
        kSpeakerFourbarAngles.put(6.5, 95.0);
        kSpeakerFourbarAngles.put(7.0, 90.0);
    }

    public static InterpolatingDoubleTreeMap kSpeakerFlywheelRPMs = new InterpolatingDoubleTreeMap();

    static {
        kSpeakerFlywheelRPMs.put(1.0, 3500.0);
        kSpeakerFlywheelRPMs.put(1.5, 4000.0);
        kSpeakerFlywheelRPMs.put(2.0, 4000.0);
        kSpeakerFlywheelRPMs.put(2.5, 4500.0);
        kSpeakerFlywheelRPMs.put(3.0, 5000.0);
        kSpeakerFlywheelRPMs.put(3.5, 5000.0);
        kSpeakerFlywheelRPMs.put(4.0, 5000.0);
        kSpeakerFlywheelRPMs.put(4.5, 5000.0);
        kSpeakerFlywheelRPMs.put(5.0, 5000.0);
        kSpeakerFlywheelRPMs.put(5.5, 5000.0);
        kSpeakerFlywheelRPMs.put(6.0, 5000.0);
        kSpeakerFlywheelRPMs.put(6.5, 5000.0);
        kSpeakerFlywheelRPMs.put(7.0, 5000.0);
    }

    public static InterpolatingDoubleTreeMap kFerryFourbarAngles = new InterpolatingDoubleTreeMap();

    static {
        kFerryFourbarAngles.put(5.3, 115.0);
        kFerryFourbarAngles.put(7.9, 120.0);
        kFerryFourbarAngles.put(9.3, 125.0);
        kFerryFourbarAngles.put(12.3, 135.0);
    }

    public static InterpolatingDoubleTreeMap kFerryFlywheelRPMs = new InterpolatingDoubleTreeMap();

    static {
        kFerryFlywheelRPMs.put(5.3, 3500.0);
        kFerryFlywheelRPMs.put(7.9, 4000.0);
        kFerryFlywheelRPMs.put(9.3, 4500.0);
        kFerryFlywheelRPMs.put(12.3, 5000.0);
    }
}
