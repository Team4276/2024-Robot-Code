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

    private boolean mIsFerry = false;
    private boolean mIsDymanic = false;
    private boolean mIsPrep = false;
    private boolean mRequestPrep = false;
    private boolean mForceDisablePrep = false;

    private ManualInput mRequestedManualInput = new ManualInput();
    private ManualInput mManualInput = new ManualInput();

    private TuningInput mTuningInput = new TuningInput();

    private double mRegressionTuningDistance = 0.0;
    private double mRegressionTuningFlywheelSetpoint = 0.0;
    private double mRegressionTuningFourbarSetpoint = 0.0;

    private double mPrevShotDistance = 0.0;
    private double mPrevShotFlywheelSetpoint = 0.0;
    private double mPrevShotFourbarSetpoint = 0.0;

    public enum GoalState {
        IDLE,
        STOW,
        INTAKE,
        READY,
        SHOOT,
        EXHAUST
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
        double flywheel_rpm = 0.0;
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

    public synchronized void idle() {
        if(mGoalState == GoalState.IDLE) return;
        mGoalState = GoalState.IDLE;

        request(idleRequest());
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

    public synchronized void stow() {
        if(mGoalState == GoalState.STOW) return;
        mGoalState = GoalState.STOW;

        if(!(!mIsPrep || mForceDisablePrep)){
            request(new ParallelRequest(
                mFlywheelSubsystem.rpmRequest(SuperstructureConstants.kSpinUpRPM),
                mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarPrepState),
                mIntakeSubsystem.stateRequest(IntakeSubsystem.State.IDLE)
            ));

            return;
        }

        request(new ParallelRequest(
            mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarStowState),
            idleRequest()
        ));
    }

    public synchronized void intake() {
        if(mGoalState == GoalState.INTAKE) return;
        mGoalState = GoalState.INTAKE;

        request(new SequentialRequest(
            new ParallelRequest(
                idleRequest(),
                mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarIntakeState),
                mIntakeSubsystem.stateRequest(IntakeSubsystem.State.INTAKE)
            ),

            breakWait(mFrontBeam, true),
            rumbleRequest(),
            new LambdaRequest(() -> mIsHoldingNote = true),
            new ParallelRequest(
                mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarPrepState),
                mIntakeSubsystem.stateRequest(IntakeSubsystem.State.IDLE)
            )
        ));
    }

    public synchronized void readyShoot() {
        if(mGoalState == GoalState.READY) return;
        mGoalState = GoalState.READY;

        if(mIsDymanic) {
            request(mIntakeSubsystem.stateRequest(IntakeSubsystem.State.IDLE));

        } else if(mIsFerry) {
            request(new SequentialRequest(
                new ParallelRequest(
                    mIntakeSubsystem.stateRequest(IntakeSubsystem.State.IDLE),
                    mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarFerryState),
                    mFlywheelSubsystem.rpmRequest(SuperstructureConstants.kFerryRPM)
                ),

                readyWait(),
                rumbleRequest()
            ));
        } else {
            request(new SequentialRequest(
                new ParallelRequest(
                    mIntakeSubsystem.stateRequest(IntakeSubsystem.State.IDLE),
                    mFourbarSubsystem.positionRequest(SuperstructureConstants.kFourbarSubCloseState),
                    mFlywheelSubsystem.rpmRequest(SuperstructureConstants.kNormalShotRPM)
                ),

                readyWait(),
                rumbleRequest()
            ));
        }
        
        
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

    public synchronized void shoot() {
        if(mGoalState == GoalState.SHOOT) return;
        mGoalState = GoalState.SHOOT;

        request(new ParallelRequest(
            mIntakeSubsystem.stateRequest(IntakeSubsystem.State.SHOOT),
            new LambdaRequest(() -> mIsHoldingNote = false),
            new LambdaRequest(() -> writeShotData())
        ));
    }

    private void writeShotData(){
        mPrevShotDistance = mRegressionTuningDistance;
        mPrevShotFlywheelSetpoint = mRegressionTuningFlywheelSetpoint;
        mPrevShotFourbarSetpoint = mRegressionTuningFourbarSetpoint;
    }

    public synchronized void exhaust() {
        if(mGoalState == GoalState.EXHAUST) return;
        mGoalState = GoalState.EXHAUST;

        request(new ParallelRequest(
            mFlywheelSubsystem.rpmRequest(0.0),
            mIntakeSubsystem.stateRequest(IntakeSubsystem.State.EXHAUST),
            new LambdaRequest(() -> mIsHoldingNote = false)
        ));
    }

    public synchronized void poop() {
        //TODO: poop
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

    public synchronized void setFerry(boolean isFerry) {
        mIsFerry = isFerry;
    }

    public synchronized void setPrep(boolean isPrep) {
        mRequestPrep = isPrep;
    }

    public synchronized void setForceDisablePrep(boolean forceDisablePrep) {
        mForceDisablePrep = forceDisablePrep;
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
        mRequestedManualInput.flywheel_voltage = voltage;
    }

    public synchronized void setManualIntakeState(IntakeSubsystem.State state) {
        mRequestedManualInput.intake_state = state;
    }

    public synchronized void setManualFourbarVoltage(double voltage) {
        mRequestedManualInput.fourbar_voltage = voltage;
    }

    public synchronized void setTuning(){
        mMode = Mode.TUNING;
    }

    public synchronized void setTuningFlywheelRPM(double RPM){
        mTuningInput.flywheel_rpm = RPM;
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
        return mGoalState == GoalState.READY && mFlywheelSubsystem.isSpunUp() && mFourbarSubsystem.atSetpoint();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mFrontBeam.update();
        mBackBeam.update();

        if (mMode == Mode.MANUAL) {
            mActiveRequest = null;
            mQueuedRequests = null;
            mManualInput = mRequestedManualInput;
            return;
        }

        mIsPrep = mRequestPrep;

        if(mGoalState != GoalState.SHOOT) {
            mPrevShotDistance = Double.NaN;
            mPrevShotFlywheelSetpoint = Double.NaN;
            mPrevShotFourbarSetpoint = Double.NaN;
        }

        mManualInput = new ManualInput();
    }

    @Override
    public synchronized void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                mQueuedRequests.clear();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized(this) {
                    try {
                        switch (mMode) {
                            case NOMINAL:
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

            mIsPrep = distance < SuperstructureConstants.kSpinUpDistance;
        }

        mRegressionTuningDistance = distance;
        mRegressionTuningFlywheelSetpoint = flywheel_setpoint;
        mRegressionTuningFourbarSetpoint = fourbar_setpoint;

        if(mGoalState == GoalState.STOW) return;

        mFlywheelSubsystem.setTargetRPM(flywheel_setpoint);
        mFourbarSubsystem.setFuseMotionSetpoint(fourbar_setpoint);
        mDriveSubsystem.feedTrackingSetpoint(drive_heading_setpoint);
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

        mFlywheelSubsystem.setTargetRPM(mTuningInput.flywheel_rpm);
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

        SmartDashboard.putBoolean("Comp/Is Prep", mIsPrep);

        SmartDashboard.putString("Comp/Superstructure Goal", mGoalState.name());
        
        SmartDashboard.putBoolean("Comp/Flywheels Spun Up", mFlywheelSubsystem.isSpunUp());

        SmartDashboard.putBoolean("Comp/Ready", isReady());
        
        SmartDashboard.putBoolean("Comp/Is Holding Note", mIsHoldingNote);

        if(Constants.disableExtraTelemetry) return;

        SmartDashboard.putNumber("Debug/Regression Tuning/Distance", mRegressionTuningDistance);
        SmartDashboard.putNumber("Debug/Regression Tuning/Flywheel Setpoint", mRegressionTuningFlywheelSetpoint);
        SmartDashboard.putNumber("Debug/Regression Tuning/Fourbar Setpoint", mRegressionTuningFourbarSetpoint);

        if(mPrevShotDistance != Double.NaN){
            SmartDashboard.putNumber("Debug/Regression Tuning/Prev Shot Distance", mPrevShotDistance);
            SmartDashboard.putNumber("Debug/Regression Tuning/Prev Shot Flywheel Setpoint", mPrevShotFlywheelSetpoint);
            SmartDashboard.putNumber("Debug/Regression Tuning/Prev Shot Fourbar Setpoint", mPrevShotFourbarSetpoint);
        }
    }
}
