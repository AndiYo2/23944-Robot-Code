package utility.managers.actions;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs multiple actions in parallel.
 * Completes when ALL contained actions complete.
 */
public class ParallelAction implements SpindexerAction {
    private final List<SpindexerAction> actions = new ArrayList<>();
    private boolean started = false;

    public void addAction(SpindexerAction action) {
        actions.add(action);
    }

    @Override
    public void start() {
        started = true;
        // Start all actions simultaneously
        for (SpindexerAction action : actions) {
            action.start();
        }
    }

    @Override
    public void update() {
        if (!started) return;

        // Update all actions
        for (SpindexerAction action : actions) {
            if (!action.isComplete()) {
                action.update();
            }
        }
    }

    @Override
    public boolean isComplete() {
        if (!started || actions.isEmpty()) return true;

        // Complete when ALL actions are complete
        for (SpindexerAction action : actions) {
            if (!action.isComplete()) {
                return false;
            }
        }
        return true;
    }

    @Override
    public void end() {
        // End all actions
        for (SpindexerAction action : actions) {
            action.end();
        }
    }

    @Override
    public String getName() {
        if (actions.isEmpty()) return "Parallel[empty]";

        StringBuilder sb = new StringBuilder("Parallel[");
        for (int i = 0; i < actions.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(actions.get(i).getName());
        }
        sb.append("]");
        return sb.toString();
    }
}
