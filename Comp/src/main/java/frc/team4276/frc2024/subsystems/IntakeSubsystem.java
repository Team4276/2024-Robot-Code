package frc.team4276.frc2024.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;

import frc.team4276.frc2024.Ports;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.rev.VIKCANSparkMax;
import frc.team4276.lib.rev.CANSparkMaxFactory;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

public class IntakeSubsystem extends Subsystem {
    private VIKCANSparkMax mMotor;

    private RelativeEncoder mRelativeEncoder;

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private State mState = State.IDLE;

    public enum State {
        IDLE(0.0),
        INTAKE(12.0),
        SLOW_FEED(5.0),
        DEFEED(-2.0),
        EXHAUST(-8.0),
        SHOOT(12.0); //TODO: add RPM control?

        public double voltage;

        State(double voltage) {
            this.voltage = voltage;
        }
    }

    private static IntakeSubsystem mInstance;

    public static synchronized IntakeSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeSubsystem();
        }
        return mInstance;
    }

    // Fill constants
    private IntakeSubsystem() {
        mMotor = CANSparkMaxFactory.createDefault(Ports.INTAKE);
        mMotor.setSmartCurrentLimit(40);
        mMotor.setWantBrakeMode(true);

        mRelativeEncoder = mMotor.getEncoder();
        mRelativeEncoder.setAverageDepth(8);
        mRelativeEncoder.setVelocityConversionFactor(5);
        
        mMotor.burnFlash();
    }
    
    public void setState(State state) {


        mState = state;
    }

    public State getState() {
        return mState;
    }

    @Override
    public void stop() {
        mState = State.IDLE;
        mPeriodicIO.voltage = 0.0;
    }

    private class PeriodicIO {
        // Inputs

        // Outputs
        double voltage;
    }

    @Override
    public void readPeriodicInputs() {}

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                setState(State.IDLE);
            }

            @Override
            public void onLoop(double timestamp) {
                mPeriodicIO.voltage = mState.voltage;
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
        SmartDashboard.putString("Intake State", mState.name());
        SmartDashboard.putNumber("Feeder RPM:", mRelativeEncoder.getVelocity());
    }
}