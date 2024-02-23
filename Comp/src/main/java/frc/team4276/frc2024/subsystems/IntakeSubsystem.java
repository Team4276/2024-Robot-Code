package frc.team4276.frc2024.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

import frc.team4276.lib.drivers.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private static IntakeSubsystem mInstance;

    private CANSparkMax mMotor;

    private DigitalInput mFrontSensor;

    private PeriodicIO mPeriodicIO;

    private IntakeState mIntakeState = IntakeState.IDLE;

    public enum IntakeState {
        IDLE(0.0),
        VOLTAGE(0.0),
        SLOWTAKE(12.0),
        SLOW_FEED(3),
        FASTAKE(12.0),
        DEFEED(-1.0),
        FOOT(12);

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
        if (mIntakeState == state)
            return;
        //TODO: check how to properly cancel states that are aleady queued

        mIntakeState = state;
        mStateStartTime = mPeriodicIO.timestamp;
    }

    public void setVoltage(double voltage) {
        mPeriodicIO.voltage = voltage;
    }

    public boolean hasNote() {
        return mPeriodicIO.front_sensor_tripped;
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
        mPeriodicIO.current_current = mMotor.getOutputCurrent();
        mPeriodicIO.front_sensor_tripped = !mFrontSensor.get();

    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setState(IntakeState.IDLE);
            }

            @Override
            public void onLoop(double timestamp) {
                switch (mIntakeState) {
                    case IDLE:
                        mPeriodicIO.voltage = mIntakeState.voltage;
                        break;

                    case VOLTAGE: break;

                    case SLOWTAKE:
                        mPeriodicIO.voltage = mIntakeState.voltage;

                        // TODO: check how long it takes for current to get to normal amps
                        if (mStateStartTime < 0.5 || mPeriodicIO.current_current <= 40) break;

                        mIntakeState = IntakeState.SLOW_FEED;

                    case SLOW_FEED:
                        if (mPeriodicIO.front_sensor_tripped) {
                            mIntakeState = IntakeState.IDLE;
                        }

                        mPeriodicIO.voltage = mIntakeState.voltage;

                        break;

                    case FASTAKE:
                        mPeriodicIO.voltage = mIntakeState.voltage;

                        if(!mPeriodicIO.front_sensor_tripped) break;

                        mIntakeState = IntakeState.DEFEED;
                        mStateStartTime = mPeriodicIO.timestamp;

                    case DEFEED:
                        //TODO: add another sensor or find how long it takes for note to stop
                        if(!mPeriodicIO.front_sensor_tripped && mPeriodicIO.timestamp > mStateStartTime + 0.25){
                            mIntakeState = IntakeState.IDLE;
                        }

                        mPeriodicIO.voltage = mIntakeState.voltage;

                        break;

                    case FOOT:
                        mPeriodicIO.voltage = mIntakeState.voltage;

                        break;

                    default:
                        break;
                }
            }

            @Override
            public void onStop(double timestamp) {
            }

        });
    }

    @Override
    public void writePeriodicOutputs() {
        mMotor.setVoltage(mPeriodicIO.voltage);
    }
}
