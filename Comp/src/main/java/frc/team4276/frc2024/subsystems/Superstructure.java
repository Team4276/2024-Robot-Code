package frc.team4276.frc2024.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team4276.frc2024.Constants.SuperstructureConstants;
import frc.team4276.frc2024.statemachines.FlywheelState;
import frc.team4276.frc2024.statemachines.SuperstructureState;
import frc.team4276.frc2024.subsystems.IntakeSubsystem.IntakeState;
import frc.team4276.lib.drivers.Subsystem;

/**
 * Use for ServoMotorSubsystems.
 * <p>
 * This architecture is only necessary for subsystems with Control States.
 */
public class Superstructure extends Subsystem {
    private final FourBarSubsystem mFourBarSubsystem;
    private final FlywheelSubsystem mFlywheelSubsystem;
    private final IntakeSubsystem mIntakeSubsystem;

    private SuperstructureState mMeasuredState;
    private SuperstructureState mCommandedState;
    private GoalState mGoalState;
    private GoalState mLatestGoalState;

    private double mDesiredFourBarVoltage = 0.0;
    private double mCommandedFourBarVoltage = 0.0;
    private boolean isFourBarVoltageControl = true;

    private FlywheelState mDesiredFlywheelState = new FlywheelState();
    private FlywheelState mCommandedFlywheelState = new FlywheelState();

    private IntakeState mDesiredIntakeState = IntakeState.IDLE;
    private double mDesiredIntakeVoltage = 0.0;
    private IntakeState mCommandedIntakeState = IntakeState.IDLE;

    public enum GoalState{
        STOW(SuperstructureConstants.kSuperstructureStowState),
        INTAKE(SuperstructureConstants.kSuperstructureIntakeState),
        SPEAKER_CLOSE_FRONT(SuperstructureConstants.kSuperstructureSpeakerCloseFrontState),
        SPEAKER_CLOSE_SIDE(SuperstructureConstants.kSuperstructureSpeakerCloseSideState),
        READY_MIDDLE(SuperstructureConstants.kSuperstructureReadyMiddleState);

        public SuperstructureState state;

        GoalState(SuperstructureState state){
            this.state = state;
        }
    } 

    private static Superstructure mInstance;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Superstructure(){
        mFourBarSubsystem = FourBarSubsystem.getInstance();
        mFlywheelSubsystem = FlywheelSubsystem.getInstance();
        mIntakeSubsystem = IntakeSubsystem.getInstance();
    }

    public void setState(GoalState state){
        if(mLatestGoalState == null) mLatestGoalState = state;

        mGoalState = state;
    }

    public void setFourBarVoltage(double voltage) {
        mDesiredFourBarVoltage = voltage;
    }

    public void setFlywheelState(FlywheelState state) {
        mDesiredFlywheelState = state;
    }

    public void setIntakeState(IntakeState state){
        mDesiredIntakeState = state;
    }

    public void setIntakeVoltage(double voltage){
        mDesiredIntakeState = IntakeState.VOLTAGE;
        mDesiredIntakeVoltage = voltage;
    }

    public void toggleFourbarVoltageMode(){
        if(isFourBarVoltageControl){
            isFourBarVoltageControl = false;
        } else {
            isFourBarVoltageControl = true;
        }
    }

    public void toggleBrakeModeOnFourbar(){
        if(mFourBarSubsystem.getIdleMode() == IdleMode.kBrake){
            mFourBarSubsystem.setIdleMode(IdleMode.kCoast);
        } else {
            mFourBarSubsystem.setIdleMode(IdleMode.kBrake);
        }
    }

    // Only place we take inputs from controlboard (other than drive subsystem);
    @Override
    public void readPeriodicInputs() {
        if (mGoalState != null) {
            mCommandedState = mGoalState.state;
        }

        mCommandedFourBarVoltage = mDesiredFourBarVoltage;
        mCommandedFlywheelState = mDesiredFlywheelState;
        mCommandedIntakeState = mDesiredIntakeState;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {
                if (isFourBarVoltageControl) {
                    mFourBarSubsystem.setVoltage(mCommandedFourBarVoltage);
                } else {
                    if(mCommandedState != null){
                        mFourBarSubsystem.setFourbarProfiledSetpoint(timestamp);
                    }
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

                if (mCommandedIntakeState == IntakeState.VOLTAGE){
                    mIntakeSubsystem.setVoltage(mDesiredIntakeVoltage);
                } else {
                    mIntakeSubsystem.setState(mCommandedIntakeState);
                }
            }

            @Override
            public void onStop(double timestamp) {}
        });
    }

    @Override
    public void writePeriodicOutputs() {}
}
