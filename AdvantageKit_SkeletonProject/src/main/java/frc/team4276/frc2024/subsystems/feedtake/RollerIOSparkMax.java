package frc.team4276.frc2024.subsystems.feedtake;

import frc.team4276.frc2024.Ports;

public class RollerIOSparkMax extends GenericRollerSystemIOSparkMax implements RollerIO {
    private static final int id = Ports.INTAKE;
    private static final int currentLimitAmps = 40;
    private static final boolean invert = false;
    private static final boolean brake = true;
    private static final double reduction = 1.0;

    public RollerIOSparkMax(){
        super(id, currentLimitAmps, invert, brake, reduction);
    }
}
