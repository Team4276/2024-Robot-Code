package frc.team4276.frc2024.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.Constants.SuperstructureConstants;
import frc.team4276.frc2024.statemachines.FlywheelState;
import frc.team4276.frc2024.statemachines.SuperstructureState;
import frc.team4276.frc2024.subsystems.IntakeSubsystem.IntakeState;
import frc.team4276.frc2024.subsystems.SimpleFourbarSubsystem.ControlState;
import frc.team4276.lib.drivers.Subsystem;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;

/**
 * Use for ServoMotorSubsystems.
 * <p>
 * This architecture is only necessary for subsystems with Control States.
 */
public class Superstructure extends Subsystem {
    private final FlywheelSubsystem mFlywheelSubsystem;
    private final IntakeSubsystem mIntakeSubsystem;
    private final SimpleFourbarSubsystem mSimpleFourbarSubsystem;

    private SuperstructureState mMeasuredState = SuperstructureState.identity();
    private SuperstructureState mCommandedState = SuperstructureState.identity();
    private GoalState mGoalState;
    private GoalState mLastGoalState;

    private double mDynamicFourbarAngle;
    private double mLatestDistance = 0.0;

    private double mFourbarScoringOffset = 0.0;

    private boolean isAutoShoot = false;
    private boolean isShooting = false;
    private double mShotStartTime = -1;

    private double mDesiredFourBarVoltage = 0.0;
    private double mCommandedFourBarVoltage = 0.0;
    private boolean isFourBarVoltageControl = false;

    private FlywheelState mDesiredFlywheelState = FlywheelState.identity();
    private FlywheelState mCommandedFlywheelState = FlywheelState.identity();

    private IntakeState mDesiredIntakeState = IntakeState.IDLE;
    private IntakeState mCommandedIntakeState = IntakeState.IDLE;

    public enum GoalState {
        STOW(new SuperstructureState(SuperstructureConstants.kFourbarStowState, IntakeState.IDLE,
                FlywheelState.identity(), false, SuperstructureState.FourbarTol.LIBERAL)),
        READY_MIDDLE(new SuperstructureState(SuperstructureConstants.kFourbarReadyMiddleState, IntakeState.IDLE,
                FlywheelState.identity(), false)),
        READY_LOW(new SuperstructureState(SuperstructureConstants.kFourbarReadyLowState, IntakeState.IDLE,
                FlywheelState.identity(), false)),
        FASTAKE(new SuperstructureState(SuperstructureConstants.kFourbarIntakeState, IntakeState.FASTAKE,
                FlywheelState.identity(), false)),
        SLOWTAKE(new SuperstructureState(SuperstructureConstants.kFourbarIntakeState, IntakeState.SLOWTAKE,
                FlywheelState.identity(), false)),
        SUB_CLOSE(new SuperstructureState(SuperstructureConstants.kFourbarSubCloseState, IntakeState.IDLE,
                SuperstructureConstants.kNormalShot, true)),
        AMP(new SuperstructureState(SuperstructureConstants.kFourbarAmpState, IntakeState.IDLE,
                SuperstructureConstants.kWhatTheFlip, true)),
        PODIUM(new SuperstructureState(SuperstructureConstants.kFourbarPodiumState, IntakeState.IDLE,
                SuperstructureConstants.kNormalShot, true)),
        DYNAMIC(new SuperstructureState(Double.NaN, IntakeState.IDLE, new FlywheelState(-4500, -4500), true));

        public SuperstructureState state;

        GoalState(SuperstructureState state) {
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

    private Superstructure() {
        mFlywheelSubsystem = FlywheelSubsystem.getInstance();
        mIntakeSubsystem = IntakeSubsystem.getInstance();
        mSimpleFourbarSubsystem = SimpleFourbarSubsystem.getInstance();
    }

    public synchronized void addFourbarScoringOffset(double delta_offset) {
        mFourbarScoringOffset += delta_offset;
    }

    public synchronized void setAutoShoot(boolean isAutoShoot) {
        this.isAutoShoot = isAutoShoot;
    }

    public synchronized void setGoalState(GoalState state) {
        if (mLastGoalState == null) {
            mLastGoalState = state;
        }

        mLastGoalState = mGoalState;
        mGoalState = state;
    }

    public synchronized void updateDynamicFourbarAngle(double fourbarAngle, double distance){
        mDynamicFourbarAngle = fourbarAngle;

        mLatestDistance = distance;

        if(mGoalState != GoalState.DYNAMIC) {
            mGoalState = GoalState.DYNAMIC;
        }
    }

    public synchronized SuperstructureState getState() {
        return mMeasuredState;
    }

    public synchronized GoalState getGoalState(){
        return mGoalState;
    }

    public synchronized void SHOOT() {
        if(mCommandedState == null) return;
        if (!mCommandedState.isShootingState) return;

        System.out.println("Fourbar Angle" + Math.toDegrees(mSimpleFourbarSubsystem.getAngleRadians()));
        System.out.println("Distance" + mLatestDistance);

        isShooting = true;
        mShotStartTime = Timer.getFPGATimestamp();
    }

    public void setFourBarVoltage(double voltage) {
        mDesiredFourBarVoltage = voltage;
    }

    public void setFlywheelState(FlywheelState state) {
        mDesiredFlywheelState = state;
    }

    public void setIntakeState(IntakeState state) {
        mDesiredIntakeState = state;
    }

    public boolean atGoal() {
        return mGoalState.state.isInRange(mMeasuredState, mSimpleFourbarSubsystem.getAngleVelRadians(), 
            SuperstructureConstants.kConservativeFourbarPositionTolerance);
    }

    public boolean atScoring(){
        if(mCommandedState == null) return false;

        return getState().isInRange(mCommandedState.fourbar_angle, mCommandedState.flywheel_state, 
            mSimpleFourbarSubsystem.getAngleVelRadians(), SuperstructureConstants.kConservativeFourbarPositionTolerance);
    }

    public boolean isHoldingNote() {
        return mMeasuredState.intake_state == IntakeState.HOLDING;
    }

    public void toggleFourbarVoltageMode() {
        if (isFourBarVoltageControl) {
            isFourBarVoltageControl = false;
            mCommandedState = mMeasuredState;
        } else {
            isFourBarVoltageControl = true;
        }
    }

    // Only place we take inputs from controlboard (other than drive subsystem);
    @Override
    public void readPeriodicInputs() {
        mMeasuredState = new SuperstructureState(mSimpleFourbarSubsystem.getAngleRadians(), mIntakeSubsystem.getState(),
                mFlywheelSubsystem.getState());

        if (mGoalState != null) {
            mCommandedState = mGoalState.state;

            if (mGoalState == GoalState.DYNAMIC) {
                if(mDynamicFourbarAngle == Double.NaN){
                    mDynamicFourbarAngle = mSimpleFourbarSubsystem.getAngleRadians();
                }
                
            } else {
                mDynamicFourbarAngle = Double.NaN;
            }

            if (mGoalState.state.isShootingState) {
                mCommandedState.fourbar_angle += mFourbarScoringOffset;
            }

            if (isAutoShoot && atScoring()) {
                SHOOT();
            }

            if (isShooting && Timer.getFPGATimestamp() < mShotStartTime + SuperstructureConstants.kAutoShotFeedTime) {
                mCommandedState.intake_state = IntakeState.FOOT;
            } else if(isShooting) {
                mCommandedState.intake_state = IntakeState.IDLE;
                isShooting = false;
            }
        }

        mCommandedFourBarVoltage = mDesiredFourBarVoltage;
        mCommandedFlywheelState = mDesiredFlywheelState.isInRange(FlywheelState.identity()) && mCommandedState != null
            ? mCommandedState.flywheel_state : mDesiredFlywheelState;
        mCommandedIntakeState = mDesiredIntakeState == IntakeState.IDLE && mCommandedState != null
             ? mCommandedState.intake_state : mDesiredIntakeState;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}

            @Override
            public void onLoop(double timestamp) {
                try{
                if (isFourBarVoltageControl) {
                    mSimpleFourbarSubsystem.setVoltage(mCommandedFourBarVoltage);

                } else if (mSimpleFourbarSubsystem.getControlState() == ControlState.CALIBRATING) {

                } else if (mCommandedState != null) {
                    mSimpleFourbarSubsystem.setSmartMotionSetpoint(mGoalState == GoalState.DYNAMIC ? 
                        mDynamicFourbarAngle : mCommandedState.fourbar_angle);
                }

                switch (mCommandedFlywheelState.desired_mode) {
                    case VOLTAGE:
                        mFlywheelSubsystem.setVoltage(mCommandedFlywheelState.top_voltage,
                                mCommandedFlywheelState.bottom_voltage);
                        break;
                    case RPM:
                        mFlywheelSubsystem.setTargetRPM(mCommandedFlywheelState.top_RPM,
                                mCommandedFlywheelState.bottom_RPM);
                        break;
                    case DEFEEDING:
                        break;
                    case WHAT_THE_FLIP:
                        mFlywheelSubsystem.setTargetRPM(mCommandedFlywheelState.top_RPM,
                                mCommandedFlywheelState.bottom_RPM);
                        mFlywheelSubsystem.setFlip();
                        break;
                    default:
                        break;
                }

                mIntakeSubsystem.setState(mCommandedIntakeState);
            } catch (Exception e) {
                System.out.println(e.getMessage());
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

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("isFourbarVoltageControl", isFourBarVoltageControl);
        SmartDashboard.putNumber("Fourbar Scoring Offset", mFourbarScoringOffset);

        SmartDashboard.putBoolean("Is Holding Note", isHoldingNote());

        if (mGoalState != null) {
            SmartDashboard.putString("Fourbar Goal", mGoalState.name());
        }
    }
}
