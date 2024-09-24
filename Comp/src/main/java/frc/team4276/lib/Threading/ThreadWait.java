package frc.team4276.lib.Threading;

import edu.wpi.first.wpilibj.Timer;

public class ThreadWait {
    public static void threadWait(double targetTime, double startTime) {
        double timeNow = Timer.getFPGATimestamp();
        double deltaTime = timeNow - startTime;
        double timeNeeded = 0.02 - deltaTime;
        while (Timer.getFPGATimestamp() - timeNow < timeNeeded) {

        }
    }

    public static void threadWait(double targetTime, double startTime, double timeNow) {
        double deltaTime = timeNow - startTime;
        double timeNeeded = 0.02 - deltaTime;
        while (Timer.getFPGATimestamp() - timeNow < timeNeeded) {

        }
    }
}
