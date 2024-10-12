package frc.team4276.frc2024;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team4276.lib.drivers.Subsystem;

import frc.team1678.lib.loops.ILooper;
import frc.team1678.lib.loops.Loop;
import frc.team1678.lib.loops.Looper;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {
    public static SubsystemManager mInstance = null;

    private List<Subsystem> mAllSubsystems;
    private List<Loop> mLoops = new ArrayList<>();

    private SubsystemManager() {}

    public static SubsystemManager getInstance() {
        if (mInstance == null) {
            mInstance = new SubsystemManager();
        }

        return mInstance;
    }

    public void outputToSmartDashboard() {
        mAllSubsystems.forEach(Subsystem::outputTelemetry);
    }

    public boolean checkSubsystems() {
        boolean ret_val = true;

        for (Subsystem s : mAllSubsystems) {
            ret_val &= s.checkSystem();
        }

        return ret_val;
    }

    public void stop() {
        mAllSubsystems.forEach(Subsystem::stop);
    }

    public List<Subsystem> getSubsystems() {
        return mAllSubsystems;
    }

    public void setSubsystems(Subsystem... allSubsystems) {
        mAllSubsystems = Arrays.asList(allSubsystems);
    }

    private double totalLoopTime = 0.0;
    private int totalLoops = 0;

    private double prevTime = 0.0;

    private class EnabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {
            mLoops.forEach(l -> l.onStart(timestamp));
            prevTime = timestamp;
        }

        @Override
        public void onLoop(double timestamp) {
            prevTime = Timer.getFPGATimestamp();
            
            mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
            mLoops.forEach(l -> l.onLoop(timestamp));
            mAllSubsystems.forEach(Subsystem::writePeriodicOutputs);

            outputToSmartDashboard();
            
            totalLoopTime += Timer.getFPGATimestamp() - prevTime;
            totalLoops++;

            if(totalLoops >= 50) {
                // SmartDashboard.putNumber("Comp/Avg Loop Time", totalLoopTime / totalLoops);
                totalLoopTime = 0.0;
                totalLoops = 0;
            }
        }

        @Override
        public void onStop(double timestamp) {
            mLoops.forEach(l -> l.onStop(timestamp));
        }
    }

    private class DisabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
            mAllSubsystems.forEach(Subsystem::readPeriodicInputs);
        }

        @Override
        public void onStop(double timestamp) {}
    }

    public void registerEnabledLoops(Looper enabledLooper) {
        mAllSubsystems.forEach(s -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        mLoops.add(loop);
    }
}
