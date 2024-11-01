package frc.team4276.frc2024.subsystems;

import com.revrobotics.RelativeEncoder;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team4276.frc2024.Constants.ClimberSubsystemConstants;
import frc.team4276.lib.drivers.ServoMotorSubsystem;
import frc.team4276.lib.rev.CANSparkMaxFactory;

public class ClimberSubsystem extends ServoMotorSubsystem {
    public enum SetpointState {
        STOW(0.0),
        LOW_RAISE(0.0),
        HIGH_RAISE(0.0);

        public double position;

        private SetpointState(double position) {
            this.position = position;
        }
    }

    private static ClimberSubsystem mInstance;

    public static ClimberSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new ClimberSubsystem();
        }

        return mInstance;
    }

    private ClimberSubsystem() {
        super(ClimberSubsystemConstants.kClimberServoConstants);

        RelativeEncoder e = CANSparkMaxFactory.configRelativeEncoder(mMaster, ClimberSubsystemConstants.kEncoderConfig);

        mMaster.getPIDController().setFeedbackDevice(e);

        mMaster.configFuseMotion(mConstants.kFuseMotionConfig, e::getPosition);

        burnFlash();
    }

    public void setSetpointState(SetpointState state) {
        setFuseMotionSetpoint(state.position);
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setVoltage(0.0);
                setWantBrakeMode(true);
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }
}
