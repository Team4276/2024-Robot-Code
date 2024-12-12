package frc.team4276.frc2024.subsystems.feedtake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: damn abstraction; fix to not look stupid
public class Feedtake extends SubsystemBase {
    public enum Goal {
        IDLE,
        INTAKE,
        FASTAKE,
        SHOOT,
        AMP,
        FAR_SHOT,
        EXHAUST
    }

    public enum GamePieceState {
        NONE,
        GRASPED,
        STAGED
    }

    private final Roller roller;
    private final RollerSensorsIO sensorsIO;
    private final RollerSensorsIOInputsAutoLogged sensorsInputs = new RollerSensorsIOInputsAutoLogged();

    private Goal goal = Goal.IDLE;
    private GamePieceState gamePieceState = GamePieceState.NONE;
    private boolean farShotFeeding = false;

    public Feedtake(Roller roller, RollerSensorsIO sensorsIO) {
        this.roller = roller;
        this.sensorsIO = sensorsIO;

    }

    @Override
    public void periodic() {
        sensorsIO.updateInputs(sensorsInputs);
        Logger.processInputs("RollersSensors", sensorsInputs);

        if (DriverStation.isDisabled()) {
            goal = Goal.IDLE;
        }

        if(sensorsInputs.front){
            gamePieceState = GamePieceState.STAGED;

        } else if(sensorsInputs.back){
            gamePieceState = GamePieceState.GRASPED;

        } else {
            gamePieceState = GamePieceState.NONE;

        }

        if(goal != Goal.FAR_SHOT){
            farShotFeeding = false;
        }

        switch (goal){
            case IDLE -> {
                roller.setGoal(Roller.Goal.IDLE);
                break;
            }
            case INTAKE -> {
                if(gamePieceState == GamePieceState.STAGED){
                    roller.setGoal(Roller.Goal.IDLE);
                    setGoal(Goal.IDLE);
                } else if(gamePieceState == GamePieceState.GRASPED){
                    roller.setGoal(Roller.Goal.SLOW_FEED);
                }

                roller.setGoal(Roller.Goal.INTAKE);
                break;
            }
            case FASTAKE -> {
                if(gamePieceState == GamePieceState.STAGED){
                    roller.setGoal(Roller.Goal.IDLE);
                    setGoal(Goal.IDLE);
                }

                roller.setGoal(Roller.Goal.INTAKE);
                break;
            }
            case SHOOT -> {
                roller.setGoal(Roller.Goal.SHOOT);
                break;
            }
            case AMP -> {
                roller.setGoal(Roller.Goal.AMP);
                break;
            }
            case FAR_SHOT -> {
                if(gamePieceState == GamePieceState.GRASPED){
                    roller.setGoal(Roller.Goal.SHOOT);
                    farShotFeeding = true;
                } else if(!farShotFeeding){
                    roller.setGoal(Roller.Goal.DEFEED);
                }
                break;
            }
            case EXHAUST -> {
                roller.setGoal(Roller.Goal.EXHAUST);
                break;
            }
        }
        
        roller.periodic();
    }

    @AutoLogOutput
    public Goal getGoal(){
        return goal;
    }

    @AutoLogOutput
    public void setGoal(Goal goal) {
        this.goal = goal;
    }

    public Command setGoalCommand(Goal goal){
        return Commands.startEnd(() -> setGoal(goal), () -> setGoal(Goal.IDLE));
    }

    @AutoLogOutput
    public GamePieceState getGamePieceState(){
        return gamePieceState;
    }

    @AutoLogOutput
    public void setGamePieceState(GamePieceState gamePieceState) {
        this.gamePieceState = gamePieceState;
    }
}
