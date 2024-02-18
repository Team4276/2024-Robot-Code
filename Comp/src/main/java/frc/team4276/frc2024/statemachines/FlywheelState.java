package frc.team4276.frc2024.statemachines;

import frc.team4276.frc2024.subsystems.FlywheelSubsystem.DesiredFlywheelMode;

public class FlywheelState {
    public DesiredFlywheelMode desired_mode = DesiredFlywheelMode.VOLTAGE;
    public double des_top_RPM = 0.0;
    public double des_bottom_RPM = 0.0;
    public double des_top_voltage = 0.0;
    public double des_bottom_voltage = 0.0;

    public FlywheelState() {
    }

    public FlywheelState(DesiredFlywheelMode desired_mode, double des_top_RPM,
            double des_bottom_RPM, double des_top_voltage, double des_bottom_voltage) {
        this.desired_mode = desired_mode;
        this.des_top_RPM = des_top_RPM;
        this.des_bottom_RPM = des_bottom_RPM;
        this.des_top_voltage = des_top_voltage;
        this.des_bottom_voltage = des_bottom_voltage;
    }

    public FlywheelState(DesiredFlywheelMode desired_mode, double des_top_RPM,
            double des_bottom_RPM) {
        this.desired_mode = desired_mode;
        this.des_top_RPM = des_top_RPM;
        this.des_bottom_RPM = des_bottom_RPM;
    }

    
}
