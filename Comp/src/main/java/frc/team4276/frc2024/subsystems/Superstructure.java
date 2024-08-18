package frc.team4276.frc2024.subsystems;

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

import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;

public class Superstructure extends Subsystem {
    private DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();
    private FlywheelSubsystem mFlywheelSubsystem = FlywheelSubsystem.getInstance();
    private IntakeSubsystem mIntakeSubsystem = IntakeSubsystem.getInstance();
    private FourbarSubsystem mFourbarSubsystem = FourbarSubsystem.getInstance();

    private BeamBreak mFrontBeam  = new BeamBreak(Ports.BEAM_FRONT);
    private BeamBreak mBackBeam = new BeamBreak(Ports.BEAM_BACK);

    private boolean mIsManual = true;

    private GoalState mRequestedState = GoalState.IDLE;
    private GoalState mGoalState = GoalState.IDLE;

    private double mScoringOffset = 0.0;
    private double mFerryOffset = 0.0;

    private boolean mIsHoldingNote = false; // TODO: test note detection reliability

    private boolean mIsFerry = false;
    private boolean mIsDymanic = false;
    private boolean mIsPrep = false;
    private boolean mRequestPrep = false;

    private ManualInput mRequestedManualInput = new ManualInput();
    private ManualInput mManualInput = new ManualInput();

    public enum GoalState {
        IDLE,
        STOW,
        INTAKE,
        READY,
        SHOOT,
        EXHAUST

    }

    private class ManualInput {
        double flywheel_voltage = 0.0;
        IntakeSubsystem.State intake_state = IntakeSubsystem.State.IDLE;
        double fourbar_voltage = 0.0;
    }

    private static Superstructure mInstance;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    public synchronized void setGoalState(GoalState state) {
        mRequestedState = state;
    }

    public synchronized void setFerry(boolean isFerry) {
        mIsFerry = isFerry;
    }

    public synchronized void setPrep(boolean isPrep) {
        mRequestPrep = isPrep;
    }

    public synchronized void setDynamic(boolean isDynamic) {
        mIsDymanic = isDynamic;
    }

    public synchronized void setManual(boolean isManual) {
        mIsManual = isManual;
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

    public synchronized void setManualFlywheelVoltage(double voltage) {
        mRequestedManualInput.flywheel_voltage = voltage;
    }

    public synchronized void setManualIntakeState(IntakeSubsystem.State state) {
        mRequestedManualInput.intake_state = state;
    }

    public synchronized void setManualFourbarVoltage(double voltage) {
        mRequestedManualInput.fourbar_voltage = voltage;
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

        mIsPrep = mRequestPrep;

        if(mGoalState != GoalState.READY) {
            hasRumbled = false;
        }

        if(mGoalState != GoalState.SHOOT) {
            mNoteDetectTime = -1.0;
        }

        if(mNoteDetectTime == -1.0) {
            mIsHoldingNote = mFrontBeam.get();
        }

        if (mIsManual) {
            mGoalState = GoalState.IDLE;
            mManualInput = mRequestedManualInput;
            return;
        }

        mGoalState = mRequestedState;
        mManualInput = new ManualInput();
    }

    @Override
    public synchronized void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                try {
                    if (mIsManual) {
                        updateManual();
                        return;
                    }

                    updateShootingSetpoints();
                    updateNomimnal(timestamp);

                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    private void updateManual() {
        mFlywheelSubsystem.setOpenLoop(mManualInput.flywheel_voltage);
        mIntakeSubsystem.setState(mManualInput.intake_state);
        mFourbarSubsystem.setVoltage(mManualInput.fourbar_voltage);
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

            mIsPrep = distance < SuperstructureConstants.kSpinUpDistance || mIsPrep;
        }

        SmartDashboard.putNumber("Debug/Regression Tuning/Distance", distance);
        SmartDashboard.putNumber("Debug/Regression Tuning/Flywheel Setpoint", flywheel_setpoint);
        SmartDashboard.putNumber("Debug/Regression Tuning/Fourbar Setpoint Degrees", Math.toDegrees(fourbar_setpoint));

        if(mGoalState == GoalState.STOW) return;

        mFlywheelSubsystem.setTargetRPM(flywheel_setpoint);
        mFourbarSubsystem.setFuseMotionSetpoint(fourbar_setpoint);
        mDriveSubsystem.feedTrackingSetpoint(drive_heading_setpoint);
    }

    private double mNoteDetectTime = -1.0;
    private boolean hasRumbled = false;

    private void updateNomimnal(double timestamp) {
        switch (mGoalState) {
            case SHOOT:
                mIntakeSubsystem.setState(IntakeSubsystem.State.SHOOT);

                if (!mIsHoldingNote)
                    break;

                if (mFrontBeam.wasCleared()) {
                    mNoteDetectTime = timestamp;
                    break;

                }

                if (mNoteDetectTime > 0.0
                        && timestamp - mNoteDetectTime > SuperstructureConstants.kShotWaitTime) {
                    mNoteDetectTime = -1.0;
                }

                break;
            case INTAKE:
                mIntakeSubsystem.setState(IntakeSubsystem.State.INTAKE);
                mFourbarSubsystem.setFuseMotionSetpoint(SuperstructureConstants.kFourbarIntakeState);

                if (mBackBeam.wasCleared()) {
                    mIntakeSubsystem.setState(IntakeSubsystem.State.SLOW_FEED);
                    break;
                }

                if (mFrontBeam.wasTripped()) {
                    mIntakeSubsystem.setState(IntakeSubsystem.State.IDLE);
                }

                break;
            case STOW:
                mIntakeSubsystem.setState(IntakeSubsystem.State.IDLE);
                mFlywheelSubsystem.setOpenLoop(0.0);
                mFourbarSubsystem.setFuseMotionSetpoint(SuperstructureConstants.kFourbarStowState);

                if (mIsPrep) {
                    mFourbarSubsystem.setFuseMotionSetpoint(SuperstructureConstants.kFourbarPrepState);

                    if (mIsHoldingNote) {
                        mFlywheelSubsystem.setOpenLoop(Constants.FlywheelConstants.kPrep);
                    }
                }

                break;
            case READY:
                if(mIsDymanic) break;

                // Setpoints from Vision are updated in updateShootingSetpoints()
                mIntakeSubsystem.setState(IntakeSubsystem.State.IDLE);
                mFlywheelSubsystem.setTargetRPM(SuperstructureConstants.kNormalShotRPM);
                mFourbarSubsystem.setFuseMotionSetpoint(SuperstructureConstants.kFourbarSubCloseState);

                if (mIsFerry) {
                    mFourbarSubsystem.setFuseMotionSetpoint(SuperstructureConstants.kFourbarFerryState);
                }

                if (mFlywheelSubsystem.isSpunUp()) {
                    if(!hasRumbled){ 
                        ControlBoard.getInstance().driver.rumble(1.0);
                        ControlBoard.getInstance().operator.rumble(1.0);
                        
                        hasRumbled = true;
                    }
                } else {
                    hasRumbled = false;

                }

                break;
            case EXHAUST:
                mIntakeSubsystem.setState(IntakeSubsystem.State.EXHAUST);
                mFourbarSubsystem.setFuseMotionSetpoint(SuperstructureConstants.kFourbarPrepState);

                break;
            case IDLE:
                mFlywheelSubsystem.setOpenLoop(0.0);
                mIntakeSubsystem.setState(IntakeSubsystem.State.IDLE);
                mFourbarSubsystem.setVoltage(0.0);

                break;

            default:
                break;
        }

    }

    @Override
    public void writePeriodicOutputs() {
    } // Leave empty

    private boolean hadNote = false;
    private boolean wasReady = false;

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putNumber("Comp/Scoring Offset", mScoringOffset);
        SmartDashboard.putNumber("Comp/Ferry Offset", mFerryOffset);

        SmartDashboard.putBoolean("Comp/Is Prep", mIsPrep);

        SmartDashboard.putString("Comp/Superstructure Goal", mGoalState.name());
        
        SmartDashboard.putBoolean("Comp/Flywheels Spun Up", mFlywheelSubsystem.isSpunUp());

        SmartDashboard.putBoolean("Comp/Ready", isReady());

        if(!wasReady && isReady()) {
            ControlBoard.getInstance().driver.rumble(1.0);
            ControlBoard.getInstance().operator.rumble(1.0);

            wasReady = true;
            
        } else if (!isReady()) {
            wasReady = false;
        }
        
        SmartDashboard.putBoolean("Comp/Is Holding Note", mIsHoldingNote);

        if (!hadNote && mIsHoldingNote) {
            ControlBoard.getInstance().driver.rumble(1.0);
            ControlBoard.getInstance().operator.rumble(1.0);

            hadNote = true;
            
        } else if (!mIsHoldingNote) {
            hadNote = false;
        }
    }
}
