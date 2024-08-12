package frc.team4276.lib.characterizations;

import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmFeedForward extends ArmFeedforward implements IFeedForward {
   public ArmFeedForward(double ks, double kg, double kv, double ka) {
      super(ks, kg, kv, ka);
   }

   public ArmFeedForward(double ks, double kg, double kv) {
      super(ks, kg, kv);
   }

   @Override
   public double calculate(double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
      return super.calculate(positionRadians, velocityRadPerSec, accelRadPerSecSquared);
   }

   @Override
   public boolean isLinear() {
       return false;
   }
}
