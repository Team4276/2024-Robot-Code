package frc.team4276.frc2024.subsystems.feedtake;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSensorsIO {
    @AutoLog
    class RollerSensorsIOInputs {
        boolean front = false;
        boolean frontTripped = false;
        boolean frontCleared = false;
        boolean back = false;
        boolean backTripped = false;
        boolean backCleared = false;
    }

    default void updateInputs(RollerSensorsIOInputs inputs) {}
    
}
