package frc.team4276.frc2024.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.Ports;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.rev.VIKCANSparkMax;
import frc.team4276.lib.rev.CANSparkMaxFactory;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

public class ClimberSubsystem extends Subsystem {
    private VIKCANSparkMax mRightMotor;
    private VIKCANSparkMax mLeftMotor;

    private DigitalInput mRightLimit;
    private DigitalInput mLeftLimit;

    private double mDesiredVoltage;

    public enum State {
        IDLE(0.0),
        RAISE(3.0),
        LOWER(-7.0),
        SLOW_LOWER(-4.0);

        public double voltage;

        State(double voltage) {
            this.voltage = voltage;
        }
    }

    private State mState = State.IDLE;
    
    private static ClimberSubsystem mInstance;

    public static ClimberSubsystem getInstance(){
        if(mInstance == null){
            mInstance = new ClimberSubsystem();
        }

        return mInstance;
    }

    private ClimberSubsystem(){
        mRightMotor = CANSparkMaxFactory.createDefault(Ports.CLIMBER_RIGHT);
        mRightMotor.setInverted(false);
        mRightMotor.setWantBrakeMode(true);
        mRightMotor.setSmartCurrentLimit(40);
        mRightMotor.burnFlash();

        mLeftMotor = CANSparkMaxFactory.createDefault(Ports.CLIMBER_LEFT);
        mLeftMotor.setInverted(true);
        mLeftMotor.setWantBrakeMode(true);
        mLeftMotor.setSmartCurrentLimit(40);
        mLeftMotor.burnFlash();

        mRightLimit = new DigitalInput(Ports.CLIMBER_LIMIT_RIGHT);
        mLeftLimit = new DigitalInput(Ports.CLIMBER_LIMIT_LEFT);

        mRightMotor.enableReverseLimit(mRightLimit::get);
        mLeftMotor.enableReverseLimit(mLeftLimit::get);
    }

    public synchronized void setDesiredState(State state){
        mState = state;
    }

    public void setWantBrakeMode(boolean brake){
        mRightMotor.setWantBrakeMode(brake);
        mLeftMotor.setWantBrakeMode(brake);
    }

    @Override
    public void stop() {
        mState = State.IDLE;
    }
    
    @Override
    public synchronized void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setWantBrakeMode(true);
            }

            @Override
            public void onLoop(double timestamp) {
                mDesiredVoltage = mState.voltage;
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    @Override
    public void writePeriodicOutputs() {
        mLeftMotor.setVoltage(mDesiredVoltage);
        mRightMotor.setVoltage(mDesiredVoltage);
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putString("Comp/Climber State", mState.toString());
        SmartDashboard.putBoolean("Debug/Right Climber Limit", mRightLimit.get());
        SmartDashboard.putBoolean("Debug/Left Climber Limit", mLeftLimit.get());
    }

}
