package frc.team4276.frc2024.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4276.frc2024.RobotState;
import frc.team4276.lib.LoggedTunableNumber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

//TODO: cleanup; refactor; add logging
public class Arm extends SubsystemBase {
    public enum Goal {
        STOW(new LoggedTunableNumber("Arm/StowDegrees", 90.0)),
        SPEAKER(RobotState.getInstance().getSpeakerAimingParameters()::getFourbarSetpoint),
        FERRY(RobotState.getInstance().getFerryAimingParameters()::getFourbarSetpoint),
        PREP(new LoggedTunableNumber("Arm/PrepDegrees", 90.0)),
        AMP(new LoggedTunableNumber("Arm/AmpDegrees", 90.0)),
        SUB(new LoggedTunableNumber("Arm/SubDegrees", 90.0)),
        PODIUM(new LoggedTunableNumber("Arm/PodiumDegrees", 90.0)),
        FERRY_FALLBACK(new LoggedTunableNumber("Arm/FerryFallbackDegrees", 90.0)),
        CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 90.0));

        private final DoubleSupplier armSetpointSupplier;

        private Goal(DoubleSupplier armSetpointSupplier) {
            this.armSetpointSupplier = armSetpointSupplier;
        }

        private double getRads() {
            return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
        }
    }

    private Goal goal = Goal.STOW;
    private double setpointRads = goal.getRads();

    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private BooleanSupplier coastOverride;

    public Arm(ArmIO io) {
        this.io = io;
        io.setBrakeMode(true);

        setDefaultCommand(setGoalCommand(Goal.STOW));
    }

    public void setCoastOverride(BooleanSupplier coastOverride){
        this.coastOverride = coastOverride;
    }

    private boolean hasFlippedCoast = false;

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (DriverStation.isDisabled()) {
            io.stop();

            if(!coastOverride.getAsBoolean()){
                hasFlippedCoast = true;
            }

            
            io.setBrakeMode(!coastOverride.getAsBoolean() && hasFlippedCoast);

        } else {
            hasFlippedCoast = false;
        }

        io.runSetpoint(setpointRads);
    }

    public Command setGoalCommand(Goal goal){
        return startEnd(() -> this.goal = goal, () -> this.goal = Goal.STOW);

    }
}
