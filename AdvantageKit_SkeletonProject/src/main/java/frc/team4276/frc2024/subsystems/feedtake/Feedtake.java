package frc.team4276.frc2024.subsystems.feedtake;

import java.util.function.DoubleSupplier;

public class Feedtake extends GenericRollerSystem<Feedtake.Goal> {

    public enum Goal implements GenericRollerSystem.VoltageGoal {
        IDLE(() -> 0.0),
        INTAKE(() -> 12.0),
        SLOW_FEED(() -> 4.0),
        DEFEED(() -> -2.0),
        EXHAUST(() -> -8.0),
        SHOOT(() -> 12.0);

        private final DoubleSupplier voltageSupplier;

        private Goal(DoubleSupplier voltageSupplier) {
            this.voltageSupplier = voltageSupplier;
        }

        @Override
        public DoubleSupplier getVoltageSupplier() {
            return voltageSupplier;
        }
    }

    private Feedtake.Goal goal = Feedtake.Goal.IDLE;

    @Override
    public Goal getGoal() {
        return goal;
    }

    public Feedtake(FeedtakeIO io) {
        super("Feedtake", io);
    }
}
