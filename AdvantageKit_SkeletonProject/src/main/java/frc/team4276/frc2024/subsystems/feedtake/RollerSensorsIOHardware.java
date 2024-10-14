package frc.team4276.frc2024.subsystems.feedtake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.team4276.frc2024.Ports;

public class RollerSensorsIOHardware implements RollerSensorsIO {
    private final DigitalInput frontTrigger;
    private final DigitalInput backTrigger;

    public RollerSensorsIOHardware(){
        frontTrigger = new DigitalInput(Ports.BEAM_FRONT);
        backTrigger = new DigitalInput(Ports.BEAM_BACK);
    }

    //TODO: check beam inputs again bc i forgot if its open by default
    @Override
    public void updateInputs(RollerSensorsIOInputs inputs) {
        inputs.frontTrigger = !frontTrigger.get();
        inputs.backTrigger = !backTrigger.get();
    }
}
