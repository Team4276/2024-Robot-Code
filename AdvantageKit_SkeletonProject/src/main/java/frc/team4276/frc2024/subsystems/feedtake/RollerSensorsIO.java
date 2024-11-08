package frc.team4276.frc2024.subsystems.feedtake;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSensorsIO {
    @AutoLog
    class RollerSensorsIOInputs {
        boolean frontTrigger = false;
        boolean backTrigger = false;
    }

    default void updateInputs(RollerSensorsIOInputs inputs) {}
    
}
