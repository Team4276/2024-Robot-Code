package frc.team4276.frc2024.auto.modes;

import edu.wpi.first.math.geometry.Pose2d;
import frc.team4276.frc2024.auto.AutoModeBase;
import frc.team4276.frc2024.auto.AutoModeEndedException;
import frc.team4276.frc2024.auto.actions.WaitAction;
import frc.team4276.frc2024.statemachines.FlywheelState;
import frc.team4276.frc2024.subsystems.FlywheelSubsystem.DesiredFlywheelMode;
import frc.team4276.frc2024.subsystems.IntakeSubsystem.IntakeState;
import frc.team4276.frc2024.subsystems.Superstructure;

public class ShootTest extends AutoModeBase {
    private final Superstructure mSuperstructure = Superstructure.getInstance();

    public ShootTest() {
    }

    @Override
    protected void routine() throws AutoModeEndedException {
        mSuperstructure.setFourBarVoltage(-2.0);
        runAction(new WaitAction(2.0));
        mSuperstructure.setFourBarVoltage(0.0);
        mSuperstructure.setFlywheelState(new FlywheelState(DesiredFlywheelMode.RPM, -3500, -3500));
        runAction(new WaitAction(2.0));
        mSuperstructure.setIntakeState(IntakeState.FOOT);
        runAction(new WaitAction(2.0));
        mSuperstructure.setIntakeState(IntakeState.IDLE);
        mSuperstructure.setFlywheelState(new FlywheelState());

    }

    @Override
    public Pose2d getStartingPose() {
        return new Pose2d();
    }
}
