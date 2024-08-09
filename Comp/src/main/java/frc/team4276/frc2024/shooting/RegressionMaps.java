package frc.team4276.frc2024.shooting;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class RegressionMaps {
    //TODO: populate
    public static InterpolatingDoubleTreeMap kSpeakerFourbarAngles = new InterpolatingDoubleTreeMap();
    static {
        kSpeakerFourbarAngles.put(0.0, 0.0);
    }

    public static InterpolatingDoubleTreeMap kSpeakerFlywheelRPMs = new InterpolatingDoubleTreeMap();
    static {
        kSpeakerFlywheelRPMs.put(0.0, 0.0);
    }

    public static InterpolatingDoubleTreeMap kFerryFourbarAngles = new InterpolatingDoubleTreeMap();
    static {
        kFerryFourbarAngles.put(0.0, 0.0);
    }

    public static InterpolatingDoubleTreeMap kFerryFlywheelRPMs = new InterpolatingDoubleTreeMap();
    static {
        kFerryFlywheelRPMs.put(0.0, 0.0);
    }
}
