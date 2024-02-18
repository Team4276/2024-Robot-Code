package frc.team4276.frc2024.subsystems;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team4276.frc2024.statemachines.FlywheelState;
import frc.team4276.lib.drivers.Subsystem;

//TODO: writeperiodic outputs

/**
 * Use for ServoMotorSubsystems.
 * <p>
 * This architecture isn't necessary if it only uses one ControlState (PID, FF,
 * etc).
 */
public class Superstructure extends Subsystem {
    private final FourBarSubsystem mFourBarSubsystem = FourBarSubsystem.getInstance();
    private final FlywheelSubsystem mFlywheelSubsystem = FlywheelSubsystem.getInstance();

    private double mDesiredVoltage = 0.0;
    private double mCommandedVoltage = 0.0;
    private boolean isFourBarVoltageControl = true;

    private FlywheelState mDesiredFlywheelState = new FlywheelState();
    private FlywheelState mCommandedFlywheelState = new FlywheelState();

    private static Superstructure mInstance;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    public void setFourBarVoltage(double des_voltage) {
        mDesiredVoltage = des_voltage;
    }

    public void setFlywheelState(FlywheelState state) {
        mDesiredFlywheelState = state;
    }

    // Only place we take inputs from controlboard (other than drive subsystem);
    @Override
    public void readPeriodicInputs() {
        mCommandedVoltage = mDesiredVoltage;
        mCommandedFlywheelState = mDesiredFlywheelState;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                if (isFourBarVoltageControl) {
                    mFourBarSubsystem.setVoltage(mCommandedVoltage);
                }

                switch (mCommandedFlywheelState.desired_mode) {
                    case VOLTAGE:
                        mFlywheelSubsystem.setVoltage(mCommandedFlywheelState.des_top_voltage,
                                mCommandedFlywheelState.des_bottom_voltage);
                        break;
                    case RPM:
                        mFlywheelSubsystem.setTargetRPM(mCommandedFlywheelState.des_top_RPM,
                                mCommandedFlywheelState.des_bottom_RPM);
                        break;
                    case DEFEEDING:
                        break;
                    case WHAT_THE_FLIP:
                        mFlywheelSubsystem.setTargetRPM(mCommandedFlywheelState.des_top_RPM,
                                mCommandedFlywheelState.des_bottom_RPM);
                        mFlywheelSubsystem.setFlip();
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
    }
}
