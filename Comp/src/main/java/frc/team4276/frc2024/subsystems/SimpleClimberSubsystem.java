package frc.team4276.frc2024.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team4276.frc2024.Ports;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team4276.lib.rev.VIKCANSparkMax;

public class SimpleClimberSubsystem extends Subsystem{
    private static SimpleClimberSubsystem mInstance;

    public static SimpleClimberSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new SimpleClimberSubsystem();
        }

        return mInstance;
    }

    private VIKCANSparkMax mRightMotor;
    private VIKCANSparkMax mLeftMotor; 

    private DigitalInput mLimitTopRight;
    private DigitalInput mLimitBotRight;
    private DigitalInput mLimitTopLeft;
    private DigitalInput mLimitBotLeft;

    private double mDesiredVoltage = 0.0;

    private SimpleClimberSubsystem() {
        mRightMotor = CANSparkMaxFactory.createDefault(Ports.CLIMBER_RIGHT);
        mRightMotor.setWantBrakeMode(true);
        mRightMotor.setInverted(false);
        mRightMotor.setSmartCurrentLimit(60);

        mLeftMotor = CANSparkMaxFactory.createDefault(Ports.CLIMBER_LEFT);
        mLeftMotor.setWantBrakeMode(true);
        mLeftMotor.setInverted(true);
        mLeftMotor.setSmartCurrentLimit(60);

        mLimitTopRight = new DigitalInput(Ports.CLIMBER_LIMIT_TOP_RIGHT);
        mLimitBotRight = new DigitalInput(Ports.CLIMBER_LIMIT_BOT_RIGHT);
        mLimitTopLeft = new DigitalInput(Ports.CLIMBER_LIMIT_TOP_LEFT);
        mLimitBotLeft = new DigitalInput(Ports.CLIMBER_LIMIT_BOT_LEFT);

        mRightMotor.burnFlash();
        mLeftMotor.burnFlash();
    }

    public void setVoltage(double volts) {
        mDesiredVoltage = volts;
    }

    public void setWantBrakeMode(boolean brake){
        mRightMotor.setWantBrakeMode(brake);
        mLeftMotor.setWantBrakeMode(brake);
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

    @Override
    public void writePeriodicOutputs() {
        double rightOutput = mDesiredVoltage;
        double leftOutput = mDesiredVoltage;

        if(mLimitTopRight != null){
            rightOutput = (!mLimitTopRight.get()) ? Math.max(rightOutput, 0.0) : rightOutput;
        }
        
        if(mLimitBotRight != null){
            rightOutput = mLimitBotRight.get() ? Math.min(rightOutput, 0.0) : rightOutput;
        }

        if(mLimitTopLeft != null){
            leftOutput = mLimitTopLeft.get() ? Math.max(leftOutput, 0.0) : leftOutput;
        }
        
        if(mLimitBotLeft != null){
            leftOutput = mLimitBotLeft.get() ? Math.min(leftOutput, 0.0) : leftOutput;
        }

        mRightMotor.setVoltage(rightOutput);
        mLeftMotor.setVoltage(leftOutput);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Debug/Climber Top Right Limit", !mLimitTopRight.get());
        SmartDashboard.putBoolean("Debug/Climber Bot Right Limit", mLimitBotRight.get());
        SmartDashboard.putBoolean("Debug/Climber Top Left Limit", mLimitTopLeft.get());
        SmartDashboard.putBoolean("Debug/Climber Bot Left Limit", mLimitBotLeft.get());
    }
}
