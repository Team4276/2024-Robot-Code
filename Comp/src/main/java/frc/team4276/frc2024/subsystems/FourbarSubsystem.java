package frc.team4276.frc2024.subsystems;

import frc.team4276.frc2024.Constants;
import frc.team4276.lib.drivers.ServoMotorSubsystem;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team4276.lib.rev.RevUtil;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

public class FourbarSubsystem extends ServoMotorSubsystem {

    private static FourbarSubsystem mInstance;

    public static FourbarSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new FourbarSubsystem(Constants.FourbarConstants.kSubsystemConstants,
                    Constants.FourbarConstants.kAbsoluteEncoderConfig);
        }

        return mInstance;
    }

    private FourbarSubsystem(ServoMotorSubsystem.ServoMotorSubsystemConstants constants,
            RevUtil.SparkAbsoluteEncoderConfig encoderConfig) {
        super(constants);

        CANSparkMaxFactory.configAbsoluteEncoder(mMaster, encoderConfig, mConstants.kFuseMotionConfig.kLooperDt);

        mPositionSupplier = mMaster.getAbsoluteEncoder()::getPosition;
        mVelocitySupplier = mMaster.getAbsoluteEncoder()::getVelocity;

        mMaster.getPIDController().setFeedbackDevice(mMaster.getAbsoluteEncoder());

        mMaster.configFuseMotion(mConstants.kFuseMotionConfig, mPositionSupplier, mVelocitySupplier);

        burnFlash();
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
            public void onLoop(double timestamp) {}

            @Override
            public void onStop(double timestamp) {}
        });
        
    }
}
