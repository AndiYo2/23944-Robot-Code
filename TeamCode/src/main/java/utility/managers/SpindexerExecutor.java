package utility.managers;

import utility.managers.actions.SpindexerAction;

import java.util.ArrayList;
import java.util.List;

/**
 * Executes a sequence of SpindexerActions.
 * Manages action lifecycle: start -> update -> isComplete -> end -> next.
 */
public class SpindexerExecutor {
    private final List<SpindexerAction> actions = new ArrayList<>();
    private int currentIndex = 0;
    private boolean running = false;

    public void addAction(SpindexerAction action) {
        actions.add(action);
    }

    public void start() {
        currentIndex = 0;
        running = true;

        if (!actions.isEmpty()) {
            actions.get(0).start();
        } else {
            running = false;
        }
    }

    public void update() {
        if (!running || actions.isEmpty()) return;

        if (currentIndex >= actions.size()) {
            running = false;
            return;
        }

        SpindexerAction current = actions.get(currentIndex);
        current.update();

        if (current.isComplete()) {
            current.end();
            currentIndex++;

            if (currentIndex < actions.size()) {
                actions.get(currentIndex).start();
            } else {
                running = false;
            }
        }
    }

    public boolean isRunning() {
        return running;
    }

    public String getCurrentActionName() {
        if (!running || currentIndex >= actions.size()) {
            return "Idle";
        }
        return actions.get(currentIndex).getName();
    }

    public void stop() {
        running = false;
    }
}
