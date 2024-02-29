package frc.team4276.frc2024.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private GoalState mLastGoalState;
    private boolean mAtGoal;

    private double mDesiredFourBarVoltage = 0.0;
    private double mCommandedFourBarVoltage = 0.0;
    private boolean isFourBarVoltageControl = true;

    private FlywheelState mDesiredFlywheelState = new FlywheelState();
    private FlywheelState mCommandedFlywheelState = new FlywheelState();

    private IntakeState mDesiredIntakeState = IntakeState.IDLE;
    private IntakeState mCommandedIntakeState = IntakeState.IDLE;

    public enum GoalState{
        STOW(SuperstructureConstants.kSuperstructureStowState),
        FASTAKE(SuperstructureConstants.kSuperstructureFastakeState),
        SLOWTAKE(SuperstructureConstants.kSuperstructureSlowtakeState),
        READY_MIDDLE(SuperstructureConstants.kSuperstructureReadyMiddleState),
        SPEAKER_CLOSE_FRONT(SuperstructureConstants.kSuperstructureSpeakerCloseFrontState),
        SPEAKER_CLOSE_SIDE(SuperstructureConstants.kSuperstructureSpeakerCloseSideState),
        SPEAKER_DYNAMIC(SuperstructureConstants.kSuperstructureDynamicSpeakerState),
        AMP_CLOSE(SuperstructureConstants.kSuperstructureAmpState),
        SCORE(SuperstructureConstants.kSuperstructureStowState); //TODO: fix scoring and intaking logic

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

    public synchronized void setGoalState(GoalState state){
        if(mLastGoalState == null) mLastGoalState = state;

        mLastGoalState = mGoalState;
        mGoalState = state;
    }

    public synchronized void setSuperstructureState(SuperstructureState state){
        
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

    public void setStateJankIntake(){
        if(mFourBarSubsystem.getMeasPosition() > 55.0){
            mDesiredFourBarVoltage = 3.0;
        }

    }

    public void setStateJankHold(){
        if(mFourBarSubsystem.getMeasPosition() < 70.0){
            mDesiredFourBarVoltage = -3.0;
        } else {
            mDesiredFourBarVoltage = 0.1;
        }

    }

    public boolean atGoal(){
        return mAtGoal;
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
                        mFourBarSubsystem.setFourBarFFSetpoint(mCommandedState.fourbar_angle);
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

                mIntakeSubsystem.setState(mCommandedIntakeState);
            }

            @Override
            public void onStop(double timestamp) {}
        });
    }

    @Override
    public void writePeriodicOutputs() {}

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("isFourbarVoltageControl", isFourBarVoltageControl);
        SmartDashboard.putString("Fourbar Goal", mGoalState == null ? "None" : mGoalState.name());
    }
}
