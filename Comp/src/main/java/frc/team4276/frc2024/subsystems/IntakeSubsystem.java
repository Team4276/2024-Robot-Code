package frc.team4276.frc2024.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.team4276.lib.drivers.Subsystem;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

public class IntakeSubsystem extends Subsystem {
    private static IntakeSubsystem mInstance;
    private CANSparkMax mMotor;
    private RelativeEncoder mRelativeEncoder;
    private DigitalInput mFrontSensor;
    private DigitalInput mBackSensor;
    private PeriodicIO mPeriodicIO;
    private IntakeState mIntakeState = IntakeState.IDLE;

    private class CurrentSensor {
        private double currentSum;
        private int count;
        private double currentRange;
        public boolean starting = true;

        CurrentSensor(double CurrentRange) {
            currentRange = CurrentRange;
        }

        public void updateCurrent(double current) {
            // check if the motor is starting to ignore the current spike as it spins up
            if (!starting) {
                currentSum += current;
                count++;
            }
        }

        // Todo: maybe use this after each cycle
        // public void averageReset() {
        //     currentSum = 0;
        //     count = 0;
        // }

        public boolean spikeCheck(double currentCurrent) {
            double averageCurrent = currentSum / count;
            if ((averageCurrent < currentCurrent) && (averageCurrent < currentCurrent - currentRange) && (!starting)) {
                return true;
            }
            return false;
        }
    }

    // don't add this to constants yet as an optimal value still needs to be found
    CurrentSensor currentSensor = new CurrentSensor(3);

    public enum IntakeState {
        IDLE(0.0),
        HOLDING(0.0),
        VOLTAGE(0.0),
        SLOWTAKE(12.0),
        SLOW_FEED(5.0),
        FASTAKE(12.0),
        DEFEED(-2.0),
        FAST_DEFEED(-8.0),
        FOOT(12),
        REVERSE(-3.0);

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
        mMotor.enableVoltageCompensation(12);
        mMotor.setSmartCurrentLimit(40);
        mMotor.setIdleMode(IdleMode.kBrake);

        mRelativeEncoder = mMotor.getEncoder();
        mRelativeEncoder.setAverageDepth(2);
        mRelativeEncoder.setVelocityConversionFactor(5);
        
        mMotor.burnFlash();
        
        mFrontSensor = new DigitalInput(1);
        mBackSensor = new DigitalInput(0);
        mPeriodicIO = new PeriodicIO();
    }
    
    public void setState(IntakeState state) {
        if(state == null) return;

        if (state != IntakeState.HOLDING && state != IntakeState.IDLE && IntakeState.VOLTAGE != state) {
            currentSensor.updateCurrent(mPeriodicIO.current_current);
        }
        if ((mIntakeState == state) || ((mIntakeState == IntakeState.HOLDING || mIntakeState == IntakeState.SLOW_FEED
                || mIntakeState == IntakeState.DEFEED) && (state != IntakeState.FOOT) && (state != IntakeState.REVERSE)
                && (state != IntakeState.FAST_DEFEED)))
            return;
        mIntakeState = state;
        mStateStartTime = mPeriodicIO.timestamp;
    }

    public void setVoltage(double voltage) {
        if (mIntakeState != IntakeState.VOLTAGE) {
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
        boolean back_sensor_tripped;

        // Outputs
        double voltage;
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.current_current = mMotor.getOutputCurrent();
        mPeriodicIO.front_sensor_tripped = !mFrontSensor.get();
        mPeriodicIO.back_sensor_tripped = !mBackSensor.get();

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
                        currentSensor.starting = true;
                        break;
                    case VOLTAGE:
                        break;
                    case HOLDING:
                        currentSensor.starting = true;
                        break;
                    case SLOWTAKE:
                        // if (mRelativeEncoder.getVelocity() >= 9500) {
                        //     currentSensor.starting = false;
                        // } else {
                        //     break;
                        // }
                        // if (currentSensor.spikeCheck(mPeriodicIO.current_current)) {
                        //     mStateStartTime = mPeriodicIO.timestamp;
                        //     mIntakeState = IntakeState.SLOW_FEED;
                        // }

                        if (mPeriodicIO.back_sensor_tripped) {
                            mStateStartTime = mPeriodicIO.timestamp;
                            mIntakeState = IntakeState.SLOW_FEED;
                        }

                    case SLOW_FEED:
                        if (mPeriodicIO.front_sensor_tripped) {
                            mIntakeState = IntakeState.HOLDING;
                        } else if (mStateStartTime - timestamp >= 4.5) {
                            mIntakeState = IntakeState.IDLE;
                        }

                        break;

                    case FASTAKE:
                        currentSensor.starting = true;
                        if (mPeriodicIO.front_sensor_tripped) {
                            mIntakeState = IntakeState.DEFEED;
                        }
                        break;
                    case DEFEED:
                        if (mPeriodicIO.front_sensor_tripped) {
                            mIntakeState = IntakeState.HOLDING;
                        }
                        break;
                    case FAST_DEFEED:
                        break;
                    case FOOT:
                        currentSensor.starting = true;
                        break;
                    case REVERSE:
                        currentSensor.starting = true;
                        break;
                    default:
                        break;
                }
                if (mIntakeState != IntakeState.VOLTAGE) {
                    mPeriodicIO.voltage = mIntakeState.voltage;
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

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Front Sensor Tripped", mPeriodicIO.front_sensor_tripped);
        SmartDashboard.putBoolean("Back Sensor Tripped", mPeriodicIO.back_sensor_tripped);
        SmartDashboard.putBoolean("Note Detetcted", currentSensor.spikeCheck(mPeriodicIO.current_current));
        SmartDashboard.putString("Intake Mode", mIntakeState.name());
        SmartDashboard.putNumber("feeder RPM:", mRelativeEncoder.getVelocity());
        SmartDashboard.putNumber("current", mPeriodicIO.current_current);
    }
}