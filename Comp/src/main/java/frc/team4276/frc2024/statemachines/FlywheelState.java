package frc.team4276.frc2024.statemachines;

import frc.team1678.lib.Util;
import frc.team4276.frc2024.Constants.FlywheelConstants;
import frc.team4276.frc2024.subsystems.FlywheelSubsystem.DesiredFlywheelMode;

public class FlywheelState {
    public DesiredFlywheelMode desired_mode = DesiredFlywheelMode.VOLTAGE;
    public double top_RPM = 0.0;
    public double bottom_RPM = 0.0;
    public double top_voltage = 0.0;
    public double bottom_voltage = 0.0;

    private static FlywheelState kIdentity = new FlywheelState();
    
    public static FlywheelState identity(){
        return kIdentity;
    }

    private FlywheelState(){}

    public FlywheelState(DesiredFlywheelMode desired_mode, double des_top_RPM,
            double des_bottom_RPM) {
        this.desired_mode = desired_mode;
        this.top_RPM = des_top_RPM;
        this.bottom_RPM = des_bottom_RPM;
    }

    public FlywheelState(double des_top_voltage, double des_bottom_voltage) {
        this.desired_mode = DesiredFlywheelMode.VOLTAGE;
        this.top_voltage = des_top_voltage;
        this.bottom_voltage = des_bottom_voltage;
    }

    public boolean isInRange(FlywheelState other){
        return Util.epsilonEquals(this.top_RPM, other.top_RPM, FlywheelConstants.kFlywheelAllowableError) &&
            Util.epsilonEquals(this.bottom_RPM, other.bottom_RPM, FlywheelConstants.kFlywheelAllowableError);
    }
    
}
