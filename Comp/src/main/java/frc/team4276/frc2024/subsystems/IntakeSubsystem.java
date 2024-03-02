package frc.team4276.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

import frc.team4276.lib.drivers.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private static IntakeSubsystem mInstance;

    private CANSparkMax mMotor;

    private DigitalInput mFrontSensor;

    private PeriodicIO mPeriodicIO;

    private IntakeState mIntakeState = IntakeState.IDLE;

    private class CurrentSensor{
        private double currentSum;
        private int count;
        private double currentRange;

        CurrentSensor(double CurrentRange){
           currentRange = CurrentRange;
        }

        public void updateCurrent(double current){
            if(!spikeCheck(current) || count < 5){
                currentSum += current;
                count++;
            }
        }

        public void averageReset(){
            currentSum = 0;
            count = 0;
        }

        public boolean spikeCheck(double currentCurrent){
            double averageCurrent = currentSum/count;
            if ((averageCurrent < currentCurrent) && (averageCurrent < currentCurrent - currentRange)){
                return true;
            }
            return false;
        }
    }
    CurrentSensor currentSensor = new CurrentSensor(3);

    public enum IntakeState {
        IDLE(0.0),
        HOLDING(0.0),
        VOLTAGE(0.0),
        SLOWTAKE(12.0),
        SLOW_FEED(3.0),
        FASTAKE(12.0),
        DEFEED(-1.5),
        FOOT(12),
        REVERSE(-0.5);

        public double voltage;

        IntakeState(double voltage) {
            this.voltage = voltage;
        }
    }

    private double mStateStartTime = 0.0;

    public static synchronized IntakeSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeSubsystem();
        }

        return mInstance;
    }

    private IntakeSubsystem() {
        mMotor = new CANSparkMax(12, MotorType.kBrushless);
        mMotor.restoreFactoryDefaults();
        mMotor.setIdleMode(IdleMode.kBrake);
        mMotor.setSmartCurrentLimit(25);
        mMotor.enableVoltageCompensation(12);
        mMotor.burnFlash();

        mFrontSensor = new DigitalInput(0);

        mPeriodicIO = new PeriodicIO();
    }

    public void setState(IntakeState state) {
        if (state != IntakeState.HOLDING && state != IntakeState.IDLE && IntakeState.VOLTAGE != state){
            currentSensor.updateCurrent(mPeriodicIO.current_current);
        }
        if ((mIntakeState == state) || ((mIntakeState == IntakeState.HOLDING || mIntakeState == IntakeState.SLOW_FEED
                || mIntakeState == IntakeState.DEFEED) && (state != IntakeState.FOOT)))
            return;

        mIntakeState = state;
        mStateStartTime = mPeriodicIO.timestamp;
    }

    public void setVoltage(double voltage) {
        if (mIntakeState != IntakeState.VOLTAGE){
            mIntakeState = IntakeState.VOLTAGE;
        }

        mPeriodicIO.voltage = voltage;
    }

    public IntakeState getState() {
        return mIntakeState;
    }

    @Override
    public void stop() {
        mIntakeState = IntakeState.IDLE;
        mPeriodicIO.voltage = 0.0;
    }

    private class PeriodicIO {
        // Inputs
        double timestamp;
        double current_current;
        boolean front_sensor_tripped;

        // Outputs
        double voltage;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.current_current = mMotor.getOutputCurrent();
        mPeriodicIO.front_sensor_tripped = !mFrontSensor.get();

    }

    private boolean hasFrontUntripped = false;

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setState(IntakeState.IDLE);
            }

            @Override
            public void onLoop(double timestamp) {
                if(currentSensor.spikeCheck(mPeriodicIO.current_current)){
                    int x = 1;
                }
                switch (mIntakeState) {
                    case IDLE: break;
                    case HOLDING: break;
                    case FASTAKE: break;
                    case VOLTAGE: break;
                    case SLOW_FEED: break;
                    case SLOWTAKE:
                        if ( mPeriodicIO.front_sensor_tripped) {
                            mIntakeState = IntakeState.DEFEED;
                        }

                        break;

                    case DEFEED:
                        if (mPeriodicIO.front_sensor_tripped) {
                            mIntakeState = IntakeState.HOLDING;
                        }
                        
                        break;

                    case FOOT: break;
                    case REVERSE: break;
                    default: break;
                }

                if(mIntakeState != IntakeState.VOLTAGE){
                    mPeriodicIO.voltage = mIntakeState.voltage;
                }
            }

            @Override
            public void onStop(double timestamp) {}

        });
    }

    @Override
    public void writePeriodicOutputs() {
        mMotor.setVoltage(mPeriodicIO.voltage);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Front Sensor Tripped", mPeriodicIO.front_sensor_tripped);
        SmartDashboard.putBoolean("Note Detetcted", currentSensor.spikeCheck(mPeriodicIO.current_current));
        SmartDashboard.putString("Intake Mode", mIntakeState.name());
    }
}
