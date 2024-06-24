package frc.team4276.lib.drivers;

public class ServoMotorSubsystem extends Subsystem {

    private ServoMotorSubsystemConstants mConstants;
    
    public static class ServoMotorSubsystemConstants{

        public FeedForwardCharacterization kFeedForwardCharacterization;
    }

    public ServoMotorSubsystem(ServoMotorSubsystemConstants constants){
        mConstants = constants;
    }

    @Override
    public void writePeriodicOutputs() {
        mConstants.kFeedForwardCharacterization.calculate(0, 0, 0);
    }



}
