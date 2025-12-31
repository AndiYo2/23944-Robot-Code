package utility;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Intake;
import subsystems.Spindexer;

/**
 * SpindexerJamClearance - Disabled (stall detection removed)
 * This class is no longer functional as stall detection has been removed from Spindexer.
 */
public class SpindexerJamClearance {
    public SpindexerJamClearance(Intake intake, Spindexer spindexer) {
        // No-op - stall detection disabled
    }

    public void periodic() {
        // No-op - stall detection disabled
    }

    public String getStatus() {
        return "Disabled (No stall detection)";
    }
}
