package frc.team4276.frc2024.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Ports;
import frc.team4276.frc2024.RobotState;
import frc.team4276.frc2024.Constants.SuperstructureConstants;
import frc.team4276.frc2024.shooting.FerryUtil;
import frc.team4276.frc2024.shooting.ShootingUtil;
import frc.team4276.frc2024.controlboard.ControlBoard;
import frc.team4276.lib.drivers.Subsystem;

import frc.team1678.lib.drivers.BeamBreak;
import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team1678.lib.requests.LambdaRequest;
import frc.team1678.lib.requests.ParallelRequest;
import frc.team1678.lib.requests.Request;
import frc.team1678.lib.requests.SequentialRequest;

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;

public class Superstructure extends Subsystem {
    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
    private FlywheelSubsystem mFlywheelSubsystem = FlywheelSubsystem.getInstance();
    private IntakeSubsystem mIntakeSubsystem = IntakeSubsystem.getInstance();
    private FourbarSubsystem mFourbarSubsystem = FourbarSubsystem.getInstance();

    private BeamBreak mFrontBeam  = new BeamBreak(Ports.BEAM_FRONT);
    private BeamBreak mBackBeam = new BeamBreak(Ports.BEAM_BACK);

    private Mode mMode = Mode.NOMINAL;

    private GoalState mGoalState = GoalState.IDLE;

    private double mScoringOffset = 0.0;
    private double mFerryOffset = 0.0;

    private boolean mIsHoldingNote = false;
    private boolean mDynamicSetpointsSet = false;
    private boolean mIsShotDoable = true;

    private boolean mIsFerry = false;
    private boolean mIsDymanic = false;
    private boolean mIsPrep = false;
    private boolean mForceDisablePrep = false;

    private ManualInput mManualInput = new ManualInput();

    private TuningInput mTuningInput = new TuningInput();

    public enum GoalState {
        IDLE,
        STOW,
        SKIM,
        INTAKE,
        READY,
        AMP,
        SHOOT,
        EXHAUST,
        POOP,
        CLIMB
    }

    public enum Mode {
        NOMINAL,
        MANUAL,
        TUNING
    }

    private class ManualInput {
        double flywheel_voltage = 0.0;
        IntakeSubsystem.State intake_state = IntakeSubsystem.State.IDLE;
        double fourbar_voltage = 0.0;
    }

    private class TuningInput {
        double flywheel_top_rpm = 0.0;
        double flywheel_bot_rpm = 0.0;
        IntakeSubsystem.State intake_state = IntakeSubsystem.State.IDLE;
        double fourbar_position = 90.0;
    }

    private static Superstructure mInstance;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    public synchronized void setNominal() {
        mMode = Mode.NOMINAL;
    }

    public synchronized void setGoalState(GoalState state) {
        mGoalState = state;
    }

    public synchronized void setPrep(boolean isPrep) {
        mIsPrep = isPrep;
    }

    public synchronized void setForceDisablePrep(boolean forceDisablePrep) {
        mForceDisablePrep = forceDisablePrep;
    }

    public synchronized void setFerry(boolean isFerry) {
        mIsFerry = isFerry;
    }

    public synchronized void setDynamic(boolean isDynamic) {
        mIsDymanic = isDynamic;
    }
    
    public synchronized void offsetScoring(double delta_offset) {
        mScoringOffset += delta_offset;
    }

    public synchronized void setScoringOffset(double offset) {
        mScoringOffset = offset;
    }

    public synchronized void offsetFerry(double delta_offset) {
        mFerryOffset += delta_offset;
    }

    public synchronized void setFerryOffset(double offset) {
        mFerryOffset = offset;
    }

    public synchronized void setManual() {
        mMode = Mode.MANUAL;
    }

    public synchronized void setManualFlywheelVoltage(double voltage) {
        mManualInput.flywheel_voltage = voltage;
    }

    public synchronized void setManualIntakeState(IntakeSubsystem.State state) {
        mManualInput.intake_state = state;
    }

    public synchronized void setManualFourbarVoltage(double voltage) {
        mManualInput.fourbar_voltage = voltage;
    }

    public synchronized void setTuning(){
        mMode = Mode.TUNING;
    }

    public synchronized void setTuningFlywheelRPM(double RPM){
        mTuningInput.flywheel_top_rpm = RPM;
        mTuningInput.flywheel_bot_rpm = RPM;
    }

    public synchronized void setTuningFlywheelRPMs(double topRpm, double botRpm){
        mTuningInput.flywheel_top_rpm = topRpm;
        mTuningInput.flywheel_bot_rpm = botRpm;
    }

    public synchronized void setTuningIntakeState(IntakeSubsystem.State state){
        mTuningInput.intake_state = state;
    }

    public synchronized void setTuningFourbarPostion(double position){
        mTuningInput.fourbar_position = position;
    }

    public synchronized boolean allRequestsComplete() {
        return mAllRequestsComplete;
    }
    
    public synchronized boolean isHoldingNote() {
        return mIsHoldingNote;
    }

    public synchronized boolean isReady() {
        if(mGoalState == GoalState.READY) {
            return mFlywheelSubsystem.isSpunUp() && 
            mFourbarSubsystem.atSetpoint(2.0) && 
            (mIsDymanic && mGoalState == GoalState.READY ? mDynamicSetpointsSet : true) &&
            mIsHoldingNote;
            
        } else if (mGoalState == GoalState.AMP) {
            return mFlywheelSubsystem.isSpunUp() && 
            mFourbarSubsystem.atSetpoint() && 
            mIsHoldingNote;
        }

        return false;
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mFrontBeam.update();
        mBackBeam.update();

        if (mMode != Mode.NOMINAL) {
            mGoalState = GoalState.IDLE;
            mActiveRequest = null;
            mQueuedRequests.clear();
            return;
        }

        if(!mIsDymanic) {
            mIsShotDoable = true;
        }
    }

    @Override
    public synchronized void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mQueuedRequests.clear();
                mActiveRequest = null;
                mHasNewRequest = false;
                mAllRequestsComplete = true;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized(this) {
                    try {
                        switch (mMode) {
                            case NOMINAL:
                                updateNominal();
                                updateRequests();
                                updateShootingSetpoints();
                                
                                break;

                            case MANUAL:
                                updateManual();

                                break;

                            case TUNING:
                                updateTuning();

                                break;
                        
                            default:
                                break;
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    private GoalState mPrevGoalState = GoalState.IDLE;
    private boolean mPrevPrepVal = false;
    private boolean mPrevForcePrepVal = false;
    private boolean mPrevFerryVal = false;

    private void updateNominal() {
        switch (mGoalState) {
            case IDLE:
                if(mPrevGoalState == mGoalState) break;

                request(idleRequest());
                
                break;
            case STOW:
                if(mPrevGoalState == mGoalState && mIsPrep == mPrevPrepVal && mPrevForcePrepVal == mForceDisablePrep) break;
                mPrevPrepVal = mIsPrep;
                mPrevForcePrepVal = mForceDisablePrep;

                if(!(!mIsPrep || mForceDisablePrep)){
                    request(new ParallelRequest(
                        mFlywheelSubsystem.rpmRequest(
                            // SuperstructureConstants.kSpinUpRPM
                            0.0
                        ), 
                        mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarPrepState),
                        mIntakeSubsystem.stateRequest(IntakeSubsystem.State.IDLE)
                    ));

                    break;
                }

                request(new ParallelRequest(
                    mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarStowState),
                    idleRequest()
                ));
                
                break;
            case SKIM:
                if(mPrevGoalState == mGoalState) break;

                request(mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarSkimState));

                break;
            case INTAKE:
                if(mPrevGoalState == mGoalState || mIsHoldingNote == true) break;

                request(new SequentialRequest(
                    new ParallelRequest(
                        mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarIntakeState),
                        mIntakeSubsystem.stateRequest(IntakeSubsystem.State.INTAKE)
                    ),
                    breakWait(mBackBeam, true),
                    
                    new ParallelRequest(
                        mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarPrepState),
                        mIntakeSubsystem.stateRequest(IntakeSubsystem.State.SLOW_FEED)
                    ),

                    breakWait(mFrontBeam, true),
                    rumbleRequest(),
                    new LambdaRequest(() -> mIsHoldingNote = true),
                    mIntakeSubsystem.stateRequest(IntakeSubsystem.State.IDLE)
                ));
                
                break;
            case READY:
                if(mPrevGoalState == mGoalState && mPrevFerryVal == mIsFerry) break;
                mPrevFerryVal = mIsFerry;

                double fourbar_position;
                double flywheel_rpm;

                if (mIsFerry) {
                    fourbar_position = SuperstructureConstants.kFourbarFerryState;
                    flywheel_rpm = SuperstructureConstants.kFerryRPM;

                } else {
                    fourbar_position = SuperstructureConstants.kFourbarSubCloseState;
                    flywheel_rpm = SuperstructureConstants.kNormalShotRPM;

                }

                request(new ParallelRequest(
                    new LambdaRequest(() -> mDynamicSetpointsSet = false),
                    mIntakeSubsystem.stateRequest(IntakeSubsystem.State.IDLE),
                    mFourbarSubsystem.positionRequest(fourbar_position),
                    mFlywheelSubsystem.rpmRequest(flywheel_rpm),
                    new SequentialRequest(
                        readyWait(),
                        rumbleRequest()
                    )
                ));
                
                break;
            case AMP:
                if(mPrevGoalState == mGoalState) break;

                request(new ParallelRequest(
                    mIntakeSubsystem.stateRequest(IntakeSubsystem.State.IDLE),
                    mFlywheelSubsystem.rpmRequest(SuperstructureConstants.kAmpTopRPM, SuperstructureConstants.kAmpBotRPM),
                    mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarAmpState),
                    new LambdaRequest(() -> mDriveSubsystem.setHeadingSetpoint(Rotation2d.fromDegrees(90.0))),
                    new SequentialRequest(
                        readyWait(),
                        rumbleRequest()
                    )
                ));

                break;
            case SHOOT:
                if(mPrevGoalState == mGoalState) break;

                if(mPrevGoalState == GoalState.AMP) {
                    request(new ParallelRequest(
                        mIntakeSubsystem.stateRequest(IntakeSubsystem.State.AMP),
                        new LambdaRequest(() -> mIsHoldingNote = false)
                    ));

                    break;
                }

                request(new ParallelRequest(
                    mIntakeSubsystem.stateRequest(IntakeSubsystem.State.SHOOT),
                    new LambdaRequest(() -> mIsHoldingNote = false)
                ));
                
                break;
            case EXHAUST:
                if(mPrevGoalState == mGoalState) break;

                request(new ParallelRequest(
                    mFlywheelSubsystem.rpmRequest(SuperstructureConstants.kExhaustRPM),
                    mIntakeSubsystem.stateRequest(IntakeSubsystem.State.EXHAUST),
                    new LambdaRequest(() -> mIsHoldingNote = false)
                ));
                
                break;
            case POOP:
                if(mPrevGoalState == mGoalState) break;

                request(new ParallelRequest(
                    mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarPoopState),
                    mFlywheelSubsystem.rpmRequest(SuperstructureConstants.kPoopTopRPM, SuperstructureConstants.kPoopBotRPM)
                    // ,
                    // new LambdaRequest(() -> mDriveSubsystem.setHeadingSetpoint(
                    //     AllianceChooser.getInstance().isAllianceRed() ? 
                    //         Rotation2d.fromDegrees(30.0) : 
                    //         Rotation2d.fromDegrees(150.0)))
                ));
                
                break;

            case CLIMB:
                if(mPrevGoalState == mGoalState) break;

                request(new ParallelRequest(
                    mFlywheelSubsystem.rpmRequest(0.0),
                    mIntakeSubsystem.stateRequest(IntakeSubsystem.State.IDLE),
                    mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarClimbState)

                ));

                break;
        
            default:
                break;
        }

        mPrevGoalState = mGoalState;
    }

    private ArrayList<Request> mQueuedRequests = new ArrayList<>();
    private Request mActiveRequest = null;
    private boolean mHasNewRequest = false;
    private boolean mAllRequestsComplete = true;

    private void updateRequests() {
        if (mHasNewRequest && mActiveRequest != null) {
            mActiveRequest.act();
            mHasNewRequest = false;
        }

        if (mActiveRequest == null) {
            if (mQueuedRequests.isEmpty()) {
                mAllRequestsComplete = true;

            } else {
                mActiveRequest = mQueuedRequests.remove(0);

            }
        } else if (mActiveRequest.isFinished()) {
            mActiveRequest = null;

        }
    }

    private void request(Request r){
        setActiveRequest(r);
        mQueuedRequests.clear();

    }

    private void setActiveRequest(Request r){
        mActiveRequest = r;
        mHasNewRequest = true;
        mAllRequestsComplete = false;

    }


    private Request idleRequest() {
        return new ParallelRequest(
                mFlywheelSubsystem.rpmRequest(0.0),
                mIntakeSubsystem.stateRequest(IntakeSubsystem.State.IDLE)
        );
    }

    private Request rumbleRequest() {
        return new Request() {
            @Override
            public void act() {
                ControlBoard.getInstance().driver.rumble(1.0);
                ControlBoard.getInstance().operator.rumble(1.0);
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    private Request readyWait(){
        return new Request() {
            @Override
            public void act() {}

            @Override
            public boolean isFinished() {
                return isReady();
            }
        };
    }

    private Request breakWait(BeamBreak b, boolean targetState){
        return new Request() {
            @Override
            public void act() {}

            @Override
            public boolean isFinished() {
                return b.get() == targetState;
            }
        };
    }
    
    private void updateShootingSetpoints() {
        if ((mGoalState != GoalState.READY && mGoalState != GoalState.SHOOT && mGoalState != GoalState.STOW) || !mIsDymanic)
            return;

        Pose2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle();

        double distance;
        double flywheel_setpoint;
        double fourbar_setpoint;
        Rotation2d drive_heading_setpoint;

        if (mIsFerry) {
            double[] params = FerryUtil.getFerryParams(robot_pose);
            distance = params[0];
            flywheel_setpoint = params[1];
            fourbar_setpoint = params[2] + mFerryOffset;
            drive_heading_setpoint = Rotation2d.fromRadians(params[3]);

            mIsPrep = true;
        } else {
            double[] params = ShootingUtil.getSpeakerShotParams(robot_pose);
            distance = params[0];
            flywheel_setpoint = params[1];
            fourbar_setpoint = params[2] + mScoringOffset;
            drive_heading_setpoint = Rotation2d.fromRadians(params[3]);

            mIsPrep = mIsPrep ? distance < SuperstructureConstants.kSpinUpDistance + 1.0: distance < SuperstructureConstants.kSpinUpDistance;
        }

        mIsShotDoable = distance < SuperstructureConstants.kDoableShotDistance;

        if(mGoalState == GoalState.STOW) return;

        mFlywheelSubsystem.setTargetRPM(flywheel_setpoint);
        mFourbarSubsystem.setFuseMotionSetpoint(fourbar_setpoint);
        mDriveSubsystem.feedTrackingSetpoint(drive_heading_setpoint);

        mDynamicSetpointsSet = true;
    }

    private void updateManual() {
        mFlywheelSubsystem.setOpenLoop(mManualInput.flywheel_voltage);
        mIntakeSubsystem.setState(mManualInput.intake_state);
        mFourbarSubsystem.setVoltage(mManualInput.fourbar_voltage);
        
    }    

    private void updateTuning() {
        Pose2d robot_pose = RobotState.getInstance().getLatestFieldToVehicle();

        double distance;

        if (mIsFerry) {
            distance = FerryUtil.getFerryParams(robot_pose)[0];

        } else {
            distance = ShootingUtil.getSpeakerShotParams(robot_pose)[0];

        }

        mFlywheelSubsystem.setTargetRPM(mTuningInput.flywheel_top_rpm, mTuningInput.flywheel_bot_rpm);
        mIntakeSubsystem.setState(mTuningInput.intake_state);
        mFourbarSubsystem.setFuseMotionSetpoint(mTuningInput.fourbar_position);

        SmartDashboard.putNumber("Debug/Test/Distance to target", distance);
    }

    @Override
    public void writePeriodicOutputs() {
    } // Leave empty

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Comp/Scoring Offset", mScoringOffset);
        SmartDashboard.putNumber("Comp/Ferry Offset", mFerryOffset);

        SmartDashboard.putString("Comp/Mode", mMode.toString());
        SmartDashboard.putBoolean("Comp/Is Prep", mIsPrep);
        SmartDashboard.putBoolean("Comp/Is Ferry", mIsFerry);
        SmartDashboard.putBoolean("Comp/Is Dynamic", mIsDymanic);
        SmartDashboard.putBoolean("Comp/Is Shot Doable", mIsShotDoable);

        SmartDashboard.putString("Comp/Superstructure Goal", mGoalState.name());

        SmartDashboard.putBoolean("Comp/Ready", isReady());
        
        SmartDashboard.putBoolean("Comp/Is Holding Note", mIsHoldingNote);

        if(Constants.disableExtraTelemetry) return;
    }
}
