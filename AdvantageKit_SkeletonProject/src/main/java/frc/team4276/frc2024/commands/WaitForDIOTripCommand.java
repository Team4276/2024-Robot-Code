package frc.team4276.frc2024.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class WaitForDIOTripCommand extends Command {
    public enum Mode {
        TRIP,
        RELEASE,
        BOTH
    }

    private final Mode mode;
    private final DigitalInput dio;

    public WaitForDIOTripCommand(DigitalInput dio, Mode mode) {
        this.mode = mode;
        this.dio = dio;
    }

    @Override
    public void initialize() {
        prevValue = dio.get();
    }

    private boolean prevValue = false;

    @Override
    public boolean isFinished() {
        boolean currValue = dio.get();

        switch (mode) {
            case TRIP:
                if (!prevValue && currValue)
                    return true;

                break;
            case RELEASE:
                if (prevValue && !currValue)
                    return true;

            case BOTH:
                if ((prevValue && !currValue) || (!prevValue && currValue))
                    return true;

            default:
                break;
        }

        prevValue = currValue;

        return false;
    }
}
