package utility.managers.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Waits for a specified duration.
 */
public class WaitAction implements SpindexerAction {
    private final double seconds;
    private final ElapsedTime timer = new ElapsedTime();

    public WaitAction(double seconds) {
        this.seconds = seconds;
    }

    @Override
    public void start() {
        timer.reset();
    }

    @Override
    public void update() {}

    @Override
    public boolean isComplete() {
        return timer.seconds() >= seconds;
    }

    @Override
    public void end() {}

    @Override
    public String getName() {
        return String.format("Wait[%.2fs]", seconds);
    }
}
