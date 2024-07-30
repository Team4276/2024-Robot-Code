package frc.team4276.frc2024.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.Ports;
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
    private DriveSubsystem mDriveSubsystem;
    private FlywheelSubsystem mFlywheelSubsystem;
    private IntakeSubsystem mIntakeSubsystem;
    private FourbarSubsystem mFourbarSubsystem;

    private BeamBreak mFrontBeam;
    private BeamBreak mBackBeam;

    private boolean mIsManual = false;

    private GoalState mRequestedState = GoalState.IDLE;
    private GoalState mGoalState = GoalState.IDLE;

    private double mScoringOffset = 0.0;

    private boolean mIsHoldingNote = false; // TODO: test note detection reliability

    private boolean mIsFerry = false;
    private boolean mIsDymanic = false;
    private boolean mIsPrep = false;

    public enum GoalState {
        IDLE,
        STOW,
        INTAKE,
        READY,
        SHOOT,
        EXHAUST

    }

    private static Superstructure mInstance;

    public static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }

        return mInstance;
    }

    private Superstructure() {
        mDriveSubsystem = DriveSubsystem.getInstance();
        mFlywheelSubsystem = FlywheelSubsystem.getInstance();
        mIntakeSubsystem = IntakeSubsystem.getInstance();
        mFourbarSubsystem = FourbarSubsystem.getInstance();

        mFrontBeam = new BeamBreak(Ports.BEAM_FRONT);
        mBackBeam = new BeamBreak(Ports.BEAM_BACK);
    }

    public void offsetScoring(double delta_offset) {
        mScoringOffset += delta_offset;
    }

    public void setGoalState(GoalState state) {
        mRequestedState = state;
    }

    public void setFerry(boolean isFerry) {
        mIsFerry = isFerry;
    }

    public void setPrep(boolean isPrep) {
        mIsPrep = isPrep;
    }

    public void setDynamic(boolean isDynamic) {
        mIsDymanic = isDynamic;
    }

    public GoalState getGoalState() {
        return mGoalState;
    }

    public boolean isHoldingNote() {
        return mIsHoldingNote;
    }

    public void overrideNoteStatus(boolean holdingNote) {
        mIsHoldingNote = holdingNote;
    }

    @Override
    public void readPeriodicInputs() {
        mFrontBeam.update();
        mBackBeam.update();

        mGoalState = mRequestedState;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
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

                    updateNomimnal(timestamp);
                    updateShootingSetpoints();

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
        // TODO: implement
    }

    private double mNoteDetectTime = -1.0;
    private boolean isSpunUp = false;

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
                        && timestamp - mNoteDetectTime > Constants.SuperstructureConstants.kShotWaitTime) {
                    mIsHoldingNote = false;
                    mNoteDetectTime = -1.0;
                }

                break;
            case INTAKE:
                mFlywheelSubsystem.setOpenLoop(0.0);
                mIntakeSubsystem.setState(IntakeSubsystem.State.INTAKE);
                mFourbarSubsystem.setFuseMotionSetpoint(Constants.SuperstructureConstants.kFourbarIntakeState);

                if (mFrontBeam.wasCleared()) {
                    mIntakeSubsystem.setState(IntakeSubsystem.State.DEFEED);
                    break;
                }

                if (mIntakeSubsystem.getState() == IntakeSubsystem.State.DEFEED && mFrontBeam.wasTripped()) {
                    mIntakeSubsystem.setState(IntakeSubsystem.State.IDLE);
                    mIsHoldingNote = true;
                }

                break;
            case STOW:
                mIntakeSubsystem.setState(IntakeSubsystem.State.IDLE);
                mFlywheelSubsystem.setOpenLoop(0.0);
                mFourbarSubsystem.setFuseMotionSetpoint(Constants.SuperstructureConstants.kFourbarStowState);

                if (mIsPrep) {
                    mFourbarSubsystem.setFuseMotionSetpoint(Constants.SuperstructureConstants.kFourbarPrepState);

                    if (mIsHoldingNote) {
                        mFlywheelSubsystem.setOpenLoop(Constants.FlywheelConstants.kPrep);
                    }
                }

                break;
            case READY:
                // Setpoints from Vision are updated in updateShootingSetpoints()
                // Defaults if Vision fails
                mIntakeSubsystem.setState(IntakeSubsystem.State.IDLE);
                mFlywheelSubsystem.setTargetRPM(Constants.SuperstructureConstants.kNormalShotRPM);
                mFourbarSubsystem.setFuseMotionSetpoint(Constants.SuperstructureConstants.kFourbarSubCloseState);

                if (mIsFerry) {
                    mFourbarSubsystem.setFuseMotionSetpoint(Constants.SuperstructureConstants.kFourbarFerryState);
                }

                if (mFlywheelSubsystem.isSpunUp() && !isSpunUp) {
                    ControlBoard.getInstance().driver.setRumble(1.0);
                    ControlBoard.getInstance().operator.setRumble(1.0);
                    isSpunUp = true;
                }

                break;
            case EXHAUST:
                mIntakeSubsystem.setState(IntakeSubsystem.State.EXHAUST);
                mFourbarSubsystem.setFuseMotionSetpoint(Constants.SuperstructureConstants.kFourbarPrepState);

                if (mBackBeam.wasCleared()) {
                    mNoteDetectTime = timestamp;

                }

                if (mNoteDetectTime > 0.0
                        && timestamp - mNoteDetectTime > Constants.SuperstructureConstants.kExhaustWaitTime) {
                    mIsHoldingNote = false;
                    mNoteDetectTime = -1.0;
                }

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

    private void updateShootingSetpoints() {
        if ((mGoalState != GoalState.READY && mGoalState != GoalState.SHOOT) || !mIsDymanic)
            return;

        Pose2d robot_pose = mDriveSubsystem.getPose();

        double flywheel_setpoint;
        double fourbar_setpoint;
        Rotation2d drive_heading_setpoint;

        if (mIsFerry) {
            double[] params = FerryUtil.getFerryParams(robot_pose);
            flywheel_setpoint = params[0];
            fourbar_setpoint = params[1];
            drive_heading_setpoint = Rotation2d.fromRadians(params[2]);
        } else {
            double[] params = ShootingUtil.getSpeakerShotParams(robot_pose);
            flywheel_setpoint = params[0];
            fourbar_setpoint = params[1];
            drive_heading_setpoint = Rotation2d.fromRadians(params[2]);
        }

        mFlywheelSubsystem.setTargetRPM(flywheel_setpoint);
        mFourbarSubsystem.setFuseMotionSetpoint(fourbar_setpoint);
        mDriveSubsystem.feedTrackingSetpoint(drive_heading_setpoint);
    }

    @Override
    public void writePeriodicOutputs() {
    }

    private boolean hadNote = false;

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Scoring Offset", mScoringOffset);

        SmartDashboard.putBoolean("Is Holding Note", mIsHoldingNote);

        if (mGoalState != null) {
            SmartDashboard.putString("Fourbar Goal", mGoalState.name());
        }

        if (!hadNote && mIsHoldingNote) {
            hadNote = true;
            ControlBoard.getInstance().driver.setRumble(1.0);
            ControlBoard.getInstance().operator.setRumble(1.0);
        } else if (!mIsHoldingNote) {
            hadNote = false;
        }
    }
}
