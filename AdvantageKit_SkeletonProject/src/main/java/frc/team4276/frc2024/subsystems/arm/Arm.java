package frc.team4276.frc2024.subsystems.arm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.team4276.frc2024.Constants;
import frc.team4276.frc2024.RobotState;
import frc.team4276.lib.LoggedTunableNumber;
import frc.team4276.lib.feedforwards.IFeedForward;

//TODO: cleanup; refactor; add viz; add characterization
public class Arm extends SubsystemBase {
    public enum Goal {
        STOW(new LoggedTunableNumber("Arm/StowDegrees", 70.0)),
        INTAKE(new LoggedTunableNumber("Arm/IntakeDegrees", 135.0)),
        SPEAKER(RobotState.getInstance().getSpeakerAimingParameters()::getFourbarSetpoint),
        FERRY(RobotState.getInstance().getFerryAimingParameters()::getFourbarSetpoint),
        PREP(new LoggedTunableNumber("Arm/PrepDegrees", 90.0)),
        AMP(new LoggedTunableNumber("Arm/AmpDegrees", 135.0)),
        SUB(new LoggedTunableNumber("Arm/SubDegrees", 135.0)),
        PODIUM(new LoggedTunableNumber("Arm/PodiumDegrees", 90.0)),
        BLIND_FERRY(new LoggedTunableNumber("Arm/BlindFerryDegrees", 135.0)),
        SKIM(new LoggedTunableNumber("Arm/SkimSetpoint", 120.0)),
        CLIMB(new LoggedTunableNumber("Arm/ClimbSetpoint", 50.0)),
        POOP(new LoggedTunableNumber("Arm/PoopSetpoint", 50.0)),
        CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 90.0));

        private final DoubleSupplier armSetpointSupplier;

        private Goal(DoubleSupplier armSetpointSupplier) {
            this.armSetpointSupplier = armSetpointSupplier;
        }

        private double getRads() {
            return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
        }
        
        private double getDegs() {
            return armSetpointSupplier.getAsDouble();
        }
    }

    @AutoLogOutput private Goal goal = Goal.STOW;
    private LoggedTunableNumber kMinPosition = new LoggedTunableNumber("Arm/MinPosition", 50.0);
    private LoggedTunableNumber kMaxPosition = new LoggedTunableNumber("Arm/MinPosition", 135.0);

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    
    private TrapezoidProfile profile;
    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
    private IFeedForward ff;

    private BooleanSupplier coastOverride;

    public Arm(ArmIO io, IFeedForward ff) {
        this.io = io;
        io.setBrakeMode(true);

        
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(150.0, 175.0));
        this.ff = ff; //TODO: test adding accel

        setDefaultCommand(setGoalCommand(Goal.STOW));
    }

    public void setCoastOverride(BooleanSupplier coastOverride){
        this.coastOverride = coastOverride;
    }

    private boolean hasFlippedCoast = false;
    private boolean wasDisabled = true;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        if (DriverStation.isDisabled()) {
            wasDisabled = true;

            io.stop();

            if(!coastOverride.getAsBoolean()){
                hasFlippedCoast = true;
            }
            
            io.setBrakeMode(!coastOverride.getAsBoolean() && hasFlippedCoast);

            setpointState = new TrapezoidProfile.State(Math.toDegrees(inputs.positionRads), 0.0);

        } else {
            if(wasDisabled){
                io.setBrakeMode(true);
                wasDisabled = false;
            }

            hasFlippedCoast = false;

            setpointState = profile.calculate(Constants.kLooperDt, setpointState,
                    new TrapezoidProfile.State(MathUtil.clamp(goal.getDegs(), kMinPosition.getAsDouble(), kMaxPosition.getAsDouble()), 0.0));

            io.runSetpoint(Math.toRadians(setpointState.position), ff.calculate(setpointState.position, setpointState.velocity, 0.0));
            Logger.recordOutput("Arm/GoalAngle", goal.getDegs());
        }
        
        Logger.recordOutput("Arm/SetpointAngle", setpointState.position);
        Logger.recordOutput("Arm/SetpointVelocity", setpointState.velocity);
        Logger.recordOutput("Arm/Goal", goal);

    }

    public Command setGoalCommand(Goal goal){
        return startEnd(() -> this.goal = goal, () -> this.goal = Goal.STOW);

    }
}
