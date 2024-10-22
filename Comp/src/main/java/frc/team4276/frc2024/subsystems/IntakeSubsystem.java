package frc.team4276.frc2024.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.Ports;
import frc.team4276.lib.drivers.Subsystem;
import frc.team4276.lib.rev.VIKCANSparkMax;
import frc.team4276.lib.rev.CANSparkMaxFactory;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team1678.lib.requests.Request;

public class IntakeSubsystem extends Subsystem {
    private VIKCANSparkMax mMotor;

    private State mState = State.IDLE;

    public enum State {
        IDLE(0.0),
        INTAKE(12.0),
        SLOW_FEED(4.0),
        DEFEED(-2.0),
        EXHAUST(-8.0),
        SHOOT(12.0);

        public double voltage;

        State(double voltage) {
            this.voltage = voltage;
        }
    }

    private static IntakeSubsystem mInstance;

    public static IntakeSubsystem getInstance() {
        if (mInstance == null) {
            mInstance = new IntakeSubsystem();
        }
        return mInstance;
    }

    private IntakeSubsystem() {
        mMotor = CANSparkMaxFactory.createDefault(Ports.INTAKE);
        mMotor.setSmartCurrentLimit(40);
        mMotor.setWantBrakeMode(true);
        
        mMotor.burnFlash();
    }

    public Request stateRequest(State s){
        return new Request(){

            @Override
            public void act() {
                setState(s);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
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
            }

            @Override
            public void onStop(double timestamp) {}
        });
    }

    @Override
    public void writePeriodicOutputs() {
        // mMotor.setVoltage(mState.voltage);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Debug/Intake State", mState.name());
        // SmartDashboard.putNumber("Debug/Intake Current", mMotor.getOutputCurrent());
        // SmartDashboard.putNumber("Debug/Intake Voltage", mMotor.getAppliedVoltage());

    }
}