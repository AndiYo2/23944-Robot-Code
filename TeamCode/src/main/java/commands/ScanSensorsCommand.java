package commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import utility.DualBallDetector;

import java.util.function.Consumer;

/**
 * Scans all three ball detector sensors and passes the results to a callback.
 * This is an instant command - it executes once and completes immediately.
 */
public class ScanSensorsCommand extends InstantCommand {

    /**
     * Container for scan results from all three sensors.
     */
    public static class ScanResults {
        public final DualBallDetector.Result sensor0;
        public final DualBallDetector.Result sensor1;
        public final DualBallDetector.Result sensor2;

        public ScanResults(DualBallDetector.Result sensor0,
                           DualBallDetector.Result sensor1,
                           DualBallDetector.Result sensor2) {
            this.sensor0 = sensor0;
            this.sensor1 = sensor1;
            this.sensor2 = sensor2;
        }
    }

    public ScanSensorsCommand(DualBallDetector sensor0,
                              DualBallDetector sensor1,
                              DualBallDetector sensor2,
                              Consumer<ScanResults> callback) {
        super(() -> {
            DualBallDetector.Result result0 = sensor0.quickCheck();
            DualBallDetector.Result result1 = sensor1.quickCheck();
            DualBallDetector.Result result2 = sensor2.quickCheck();
            callback.accept(new ScanResults(result0, result1, result2));
        });
    }
}
