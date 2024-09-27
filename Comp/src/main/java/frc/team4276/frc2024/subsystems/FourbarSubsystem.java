package frc.team4276.frc2024.subsystems;

import frc.team4276.frc2024.Constants.FourbarConstants;
import frc.team4276.lib.drivers.ServoMotorSubsystem;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team4276.lib.rev.RevUtil;

import com.revrobotics.AbsoluteEncoder;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

public class FourbarSubsystem extends ServoMotorSubsystem {

    private static FourbarSubsystem mInstance;

    public static FourbarSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new FourbarSubsystem(FourbarConstants.kSubsystemConstants,
                    FourbarConstants.kFourbarEncoderConfig);
        }

        return mInstance;
    }

    private FourbarSubsystem(ServoMotorSubsystem.ServoMotorSubsystemConstants constants,
            RevUtil.SparkAbsoluteEncoderConfig encoderConfig) {
        super(constants);

        AbsoluteEncoder e = CANSparkMaxFactory.configAbsoluteEncoder(mMaster, encoderConfig);

        mMaster.getPIDController().setFeedbackDevice(mMaster.getAbsoluteEncoder());

        mMaster.configFuseMotion(mConstants.kFuseMotionConfig, e::getPosition, e::getVelocity);

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

    @Override
    public double getPosition() {
        return mMaster.getAbsoluteEncoder().getPosition();
    }
    
    @Override
    public double getVelocity() {
        return mMaster.getAbsoluteEncoder().getVelocity();
    }


}
