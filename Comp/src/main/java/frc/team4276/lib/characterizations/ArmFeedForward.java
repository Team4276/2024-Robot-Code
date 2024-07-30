package frc.team4276.lib.characterizations;

public class ArmFeedForward implements IFeedForward {
   public final double ks;
   public final double kg;
   public final double kv;
   public final double ka;

   public ArmFeedForward(double ks, double kg, double kv, double ka) {
      this.ks = ks;
      this.kg = kg;
      this.kv = kv;
      this.ka = ka;
      if (kv < 0.0) {
         throw new IllegalArgumentException("kv must be a non-negative number, got " + kv + "!");
      } else if (ka < 0.0) {
         throw new IllegalArgumentException("ka must be a non-negative number, got " + ka + "!");
      }
   }

   public ArmFeedForward(double ks, double kg, double kv) {
      this(ks, kg, kv, 0.0);
   }

   @Override
   public double calculate(double positionRadians, double velocityRadPerSec, double accelRadPerSecSquared) {
      return this.ks * Math.signum(velocityRadPerSec) + this.kg * Math.cos(positionRadians) + this.kv * velocityRadPerSec + this.ka * accelRadPerSecSquared;
   }

   public double maxAchievableVelocity(double maxVoltage, double angle, double acceleration) {
      return (maxVoltage - this.ks - Math.cos(angle) * this.kg - acceleration * this.ka) / this.kv;
   }

   public double minAchievableVelocity(double maxVoltage, double angle, double acceleration) {
      return (-maxVoltage + this.ks - Math.cos(angle) * this.kg - acceleration * this.ka) / this.kv;
   }

   public double maxAchievableAcceleration(double maxVoltage, double angle, double velocity) {
      return (maxVoltage - this.ks * Math.signum(velocity) - Math.cos(angle) * this.kg - velocity * this.kv) / this.ka;
   }

   public double minAchievableAcceleration(double maxVoltage, double angle, double velocity) {
      return this.maxAchievableAcceleration(-maxVoltage, angle, velocity);
   }
}
