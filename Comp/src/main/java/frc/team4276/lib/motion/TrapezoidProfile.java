package frc.team4276.lib.motion;

// Based on WPI class; minus some of the object oriented aspects
public class TrapezoidProfile {
   private int m_direction;
   private final double m_maxAccel;
   private final double m_maxVel;
   private double[] m_initial; // {pos, vel}
   private double m_endAccel;
   private double m_endFullSpeed;
   private double m_endDeccel;

   public TrapezoidProfile(double maxVel, double maxAccel) {
      m_maxVel = maxVel;
      m_maxAccel = maxAccel;
   }

   public double[] calculate(double t, double[] initial, double[] goal) {
      this.m_direction = shouldFlipAcceleration(initial[0], goal[0]) ? -1 : 1;
      this.m_initial = this.direct(initial);
      goal = this.direct(goal);
      if (this.m_initial[1] > this.m_maxVel) {
         this.m_initial[1] = this.m_maxVel;
      }

      double cutoffBegin = this.m_initial[1] / this.m_maxAccel;
      double cutoffDistBegin = cutoffBegin * cutoffBegin * this.m_maxAccel / 2.0;
      double cutoffEnd = goal[1] / this.m_maxAccel;
      double cutoffDistEnd = cutoffEnd * cutoffEnd * this.m_maxAccel / 2.0;
      double fullTrapezoidDist = cutoffDistBegin + (goal[0] - this.m_initial[0]) + cutoffDistEnd;
      double accelerationTime = this.m_maxVel / this.m_maxAccel;
      double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * this.m_maxAccel;
      if (fullSpeedDist < 0.0) {
         accelerationTime = Math.sqrt(fullTrapezoidDist / this.m_maxAccel);
         fullSpeedDist = 0.0;
      }

      this.m_endAccel = accelerationTime - cutoffBegin;
      this.m_endFullSpeed = this.m_endAccel + fullSpeedDist / this.m_maxVel;
      this.m_endDeccel = this.m_endFullSpeed + accelerationTime - cutoffEnd;
      double[] result = {this.m_initial[0], this.m_initial[1]};
      if (t < this.m_endAccel) {
         result[1] += t * this.m_maxAccel;
         result[0] += (this.m_initial[1] + t * this.m_maxAccel / 2.0) * t;
      } else if (t < this.m_endFullSpeed) {
         result[1] = this.m_maxVel;
         result[0] += (this.m_initial[1] + this.m_endAccel * this.m_maxAccel / 2.0) * this.m_endAccel + this.m_maxVel * (t - this.m_endAccel);
      } else if (t <= this.m_endDeccel) {
         result[1] = goal[1] + (this.m_endDeccel - t) * this.m_maxAccel;
         double timeLeft = this.m_endDeccel - t;
         result[0] = goal[0] - (goal[1] + timeLeft * this.m_maxAccel / 2.0) * timeLeft;
      } else {
         result = goal;
      }

      return this.direct(result);
   }

   public double timeLeftUntil(double target) {
      double position = this.m_initial[0] * (double)this.m_direction;
      double velocity = this.m_initial[1] * (double)this.m_direction;
      double endAccel = this.m_endAccel * (double)this.m_direction;
      double endFullSpeed = this.m_endFullSpeed * (double)this.m_direction - endAccel;
      if (target < position) {
         endAccel = -endAccel;
         endFullSpeed = -endFullSpeed;
        velocity = -velocity;
      }

      endAccel = Math.max(endAccel, 0.0);
      endFullSpeed = Math.max(endFullSpeed, 0.0);
      double acceleration = this.m_maxAccel;
      double decceleration = -this.m_maxAccel;
      double distToTarget = Math.abs(target - position);
      if (distToTarget < 1.0E-6) {
         return 0.0;
      } else {
         double accelDist = velocity * endAccel + 0.5 * acceleration * endAccel * endAccel;
         double deccelVelocity;
         if (endAccel > 0.0) {
            deccelVelocity = Math.sqrt(Math.abs(velocity * velocity + 2.0 * acceleration * accelDist));
         } else {
            deccelVelocity = velocity;
         }

         double fullSpeedDist = this.m_maxVel * endFullSpeed;
         double deccelDist;
         if (accelDist > distToTarget) {
            accelDist = distToTarget;
            fullSpeedDist = 0.0;
            deccelDist = 0.0;
         } else if (accelDist + fullSpeedDist > distToTarget) {
            fullSpeedDist = distToTarget - accelDist;
            deccelDist = 0.0;
         } else {
            deccelDist = distToTarget - fullSpeedDist - accelDist;
         }

         double accelTime = (-velocity + Math.sqrt(Math.abs(velocity * velocity + 2.0 * acceleration * accelDist))) / acceleration;
         double deccelTime = (-deccelVelocity + Math.sqrt(Math.abs(deccelVelocity * deccelVelocity + 2.0 * decceleration * deccelDist))) / decceleration;
         double fullSpeedTime = fullSpeedDist / this.m_maxVel;
         return accelTime + fullSpeedTime + deccelTime;
      }
   }

   public double totalTime() {
      return this.m_endDeccel;
   }

   public boolean isFinished(double t) {
      return t >= this.totalTime();
   }

   private static boolean shouldFlipAcceleration(double initial, double goal) {
      return initial > goal;
   }

   private double[] direct(double[] in) {
      double[] result = {in[0], in[1]};
      result[0] *= (double)this.m_direction;
      result[1] *= (double)this.m_direction;
      return result;
   }
}
