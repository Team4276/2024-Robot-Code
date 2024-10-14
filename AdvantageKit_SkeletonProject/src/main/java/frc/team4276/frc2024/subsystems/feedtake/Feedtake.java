package frc.team4276.frc2024.subsystems.feedtake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.team4276.frc2024.subsystems.feedtake.Roller.Goal;

//TODO: damn abstraction; fix to not look stupid
public class Feedtake extends SubsystemBase {
    private final Roller roller;
    private final RollerSensorsIO sensorsIO;
    private final RollerSensorsIOInputsAutoLogged sensorsInputs = new RollerSensorsIOInputsAutoLogged();

    private Roller.Goal goal = Goal.IDLE;

    public Feedtake(Roller roller, RollerSensorsIO sensorsIO) {
        this.roller = roller;
        this.sensorsIO = sensorsIO;

        setDefaultCommand(setGoalCommand(Roller.Goal.IDLE));
    }

    @Override
    public void periodic() {
        sensorsIO.updateInputs(sensorsInputs);

        if (DriverStation.isDisabled()) {
            goal = Goal.IDLE;
        }

        roller.setGoal(goal);
        
        roller.periodic();
    }

    public Roller.Goal getGoal(){
        return goal;
    }

    private void setGoal(Roller.Goal goal) {
        this.goal = goal;
    }

    public Command setGoalCommand(Roller.Goal goal) {
        return startEnd(() -> setGoal(goal), () -> setGoal(Roller.Goal.IDLE));
    }

    public Command intakeCommand() {
        return new SequentialCommandGroup(
            run(() -> setGoal(Roller.Goal.INTAKE)).until(() -> sensorsInputs.backTrigger),
            run(() -> setGoal(Roller.Goal.SLOW_FEED)).until(() -> sensorsInputs.frontTrigger)
        );
    }

    public Command farShotCommand() {
        return new SequentialCommandGroup(
            run(() -> setGoal(Roller.Goal.DEFEED)).until(() -> !sensorsInputs.frontTrigger),
            new WaitUntilCommand(() -> sensorsInputs.backTrigger),
            setGoalCommand(Goal.SHOOT)
        );
    }

    public Command farShotStageCommand() {
        return new SequentialCommandGroup(
            run(() -> setGoal(Roller.Goal.DEFEED)).until(() -> !sensorsInputs.frontTrigger),
            new WaitUntilCommand(() -> sensorsInputs.backTrigger)
        );
    }


}
