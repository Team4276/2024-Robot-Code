package frc.team4276.frc2024.subsystems;

import com.revrobotics.RelativeEncoder;

import frc.team4276.frc2024.Constants.ClimberSubsystemConstants;
import frc.team4276.lib.drivers.ServoMotorSubsystem;
import frc.team4276.lib.rev.CANSparkMaxFactory;

public class CilmberSubsystem extends ServoMotorSubsystem {
    public enum SetpointState {
        STOW(0.0),
        LOW_RAISE(0.0),
        HIGH_RAISE(0.0);

        public double position;

        private SetpointState(double position){
            this.position = position;
        }
    }

    private static CilmberSubsystem mInstance;

    public static CilmberSubsystem getInstance(){
        if(mInstance == null){
            mInstance = new CilmberSubsystem();
        }

        return mInstance;
    }

    private CilmberSubsystem(){
        super(ClimberSubsystemConstants.kClimberServoConstants);
        
        RelativeEncoder e = CANSparkMaxFactory.configRelativeEncoder(mMaster, ClimberSubsystemConstants.kEncoderConfig);

        mMaster.getPIDController().setFeedbackDevice(e);

        mMaster.configFuseMotion(mConstants.kFuseMotionConfig, e::getPosition);

        burnFlash();
    }

    public void setSetpointState(SetpointState state){
        setFuseMotionSetpoint(state.position);
    }

}
