package frc.team4276.frc2024.subsystems;

import com.revrobotics.SparkAbsoluteEncoder;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team4276.frc2024.Constants;
import frc.team4276.lib.drivers.ServoMotorSubsystem;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team4276.lib.rev.RevUtil;

public class FourbarSubsystem extends ServoMotorSubsystem {
    private SparkAbsoluteEncoder mAbsoluteEncoder;

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

        mAbsoluteEncoder = mMaster.getAbsoluteEncoder();
        mAbsoluteEncoder.setInverted(encoderConfig.kIsInverted);
        mAbsoluteEncoder.setPositionConversionFactor(encoderConfig.kUnitsPerRotation);
        mAbsoluteEncoder.setVelocityConversionFactor(encoderConfig.kUnitsPerRotation);
        mAbsoluteEncoder.setZeroOffset(encoderConfig.kOffset);
        mAbsoluteEncoder.setAverageDepth(encoderConfig.kAvgSamplingDepth);

        mPositionSupplier = mAbsoluteEncoder::getPosition;
        mVelocitySupplier = mAbsoluteEncoder::getVelocity;

        mMaster.getPIDController().setFeedbackDevice(mAbsoluteEncoder);

        mMaster.configFuseMotion(mConstants.kFuseMotionConfig, mPositionSupplier, mVelocitySupplier);

        CANSparkMaxFactory.configAbsoluteEncoder(mMaster, mConstants.kFuseMotionConfig.kLooperDt);

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
