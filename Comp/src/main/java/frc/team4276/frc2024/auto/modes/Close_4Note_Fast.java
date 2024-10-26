package frc.team4276.frc2024.auto.modes;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.team254.lib.geometry.Pose2d;
import frc.team254.lib.geometry.Rotation2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.SwerveTrajectoryAction;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.auto.actions.WaitForAction;
import frc.team4276.frc2024.field.AllianceChooser;
import frc.team4276.frc2024.subsystems.DriveSubsystem;
import frc.team4276.frc2024.subsystems.Superstructure;
import frc.team4276.frc2024.subsystems.Superstructure.GoalState;

public class Close_4Note_Fast extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();
    private final DriveSubsystem mDriveSubsystem = DriveSubsystem.getInstance();

    private final SwerveTrajectoryAction traj1;
    private final SwerveTrajectoryAction traj2;
    private final SwerveTrajectoryAction traj3;
    private final SwerveTrajectoryAction traj4;

    public Close_4Note_Fast(){
        traj1 = new SwerveTrajectoryAction("Close_4Note_Fast", 1);
        traj2 = new SwerveTrajectoryAction("Close_4Note_Fast", 2);
        traj3 = new SwerveTrajectoryAction("Close_4Note_Fast", 3);
        traj4 = new SwerveTrajectoryAction("Close_4Note_Fast", 4);
    }

    //TODO: add auto wait time for shots in superstructure logic
    private double kShotWaitTime = 0.5;
    private double kReadyWaitTime = 1.0;

    @Override
    protected void routine() throws AutoModeEndedException {
        // boolean isRed = AllianceChooser.getInstance().isAllianceRed();

        // Set Control States
        mSuperstructure.setDynamic(true);
        mSuperstructure.setNominal();
        mSuperstructure.setPrep(true);

        // Note 1 Shoot
        mSuperstructure.setGoalState(GoalState.READY);
        mDriveSubsystem.overrideHeading(true);
        runAction(new WaitAction(kReadyWaitTime));
        mSuperstructure.setGoalState(GoalState.SHOOT);
        runAction(new WaitAction(kShotWaitTime));
        mSuperstructure.setGoalState(GoalState.SKIM);
        mDriveSubsystem.overrideHeading(false);
        // mDriveSubsystem.setHeadingSetpoint(isRed ? Rotation2d.fromDegrees(180.0) : 
        //     Rotation2d.fromDegrees(0.0)); //TODO: fix this its jank as fuck

        // Note 2 Drive and Pickup and Drive to Score
        runAction(traj1);
        mSuperstructure.setGoalState(GoalState.INTAKE);
        runAction(traj2);
        
        // Note 2 Score
        mSuperstructure.setGoalState(GoalState.READY);
        runAction(new WaitAction(kReadyWaitTime));
        mSuperstructure.setGoalState(GoalState.SHOOT);
        runAction(new WaitAction(kShotWaitTime));
        mSuperstructure.setGoalState(GoalState.INTAKE);

        // Note 3 Drive and Pickup and Score
        runAction(traj3);
        runAction(new WaitForAction(mSuperstructure::isHoldingNote));
        mSuperstructure.setGoalState(GoalState.READY);
        runAction(new WaitAction(kReadyWaitTime));
        mSuperstructure.setGoalState(GoalState.SHOOT);
        runAction(new WaitAction(kShotWaitTime));
        mSuperstructure.setGoalState(GoalState.SKIM);

        // Note 4 Drive and Pickup


        // Note 4 Drive and Shoot
        
        mSuperstructure.setPrep(false);

    }

    @Override
    public Pose2d getStartingPose() {
        return traj1.getInitialPose();
    }
    
}
