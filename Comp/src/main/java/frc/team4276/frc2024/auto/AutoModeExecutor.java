package frc.team4276.frc2024.auto;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous
 * mode.
 */
public class AutoModeExecutor {
    private AutoModeBase m_auto_mode;

    public void setAutoMode(AutoModeBase new_auto_mode) {
        m_auto_mode = new_auto_mode;
    }

    public void start() {
        if (m_auto_mode != null) {
            m_auto_mode.run();
        }
    }

    public void stop() {
        if (m_auto_mode != null) {
            m_auto_mode.stop();
        }
    }

}
