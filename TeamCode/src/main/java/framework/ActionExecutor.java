package framework;

import java.util.ArrayList;
import java.util.List;

/**
 * Central executor that manages the lifecycle of a sequence of actions.
 * This replaces manual state machine switching with automatic action progression.
 */
public class ActionExecutor {
    private final List<Action> actions;
    private int currentActionIndex;
    private boolean started;
    private boolean finished;

    public ActionExecutor() {
        this.actions = new ArrayList<>();
        this.currentActionIndex = 0;
        this.started = false;
        this.finished = false;
    }

    /**
     * Adds an action to the execution sequence.
     *
     * @param action the action to add
     */
    public void addAction(Action action) {
        actions.add(action);
    }

    /**
     * Starts the executor, beginning the first action.
     * Should be called from the autonomous start() method.
     */
    public void start() {
        started = true;
        finished = false;
        currentActionIndex = 0;

        if (!actions.isEmpty()) {
            actions.get(0).start();
        }
    }

    /**
     * Updates the current action and advances to the next when complete.
     * Should be called from the autonomous loop() method.
     */
    public void update() {
        if (!started || finished) {
            return;
        }

        if (currentActionIndex >= actions.size()) {
            finished = true;
            return;
        }

        Action currentAction = actions.get(currentActionIndex);
        currentAction.update();

        if (currentAction.isComplete()) {
            currentAction.end();
            currentActionIndex++;

            // Start the next action if there is one
            if (currentActionIndex < actions.size()) {
                actions.get(currentActionIndex).start();
            } else {
                finished = true;
            }
        }
    }

    /**
     * Returns whether the executor is currently executing actions.
     *
     * @return true if executing, false if not started or finished
     */
    public boolean isExecuting() {
        return started && !finished;
    }

    /**
     * Returns whether all actions have completed.
     *
     * @return true if finished, false otherwise
     */
    public boolean isFinished() {
        return finished;
    }

    /**
     * Returns the current action being executed, or null if none.
     *
     * @return the current action or null
     */
    public Action getCurrentAction() {
        if (currentActionIndex >= 0 && currentActionIndex < actions.size()) {
            return actions.get(currentActionIndex);
        }
        return null;
    }

    /**
     * Returns the total number of actions in this executor.
     *
     * @return the action count
     */
    public int getActionCount() {
        return actions.size();
    }

    /**
     * Returns the current action index (0-based).
     *
     * @return the current index
     */
    public int getCurrentActionIndex() {
        return currentActionIndex;
    }

    /**
     * Returns a formatted status string for telemetry display.
     *
     * @return formatted status string showing current action and progress
     */
    public String getStatusString() {
        if (!started) {
            return "Not started";
        }
        if (finished) {
            return "Finished";
        }
        Action currentAction = getCurrentAction();
        if (currentAction == null) {
            return "No current action";
        }
        return String.format("[%d/%d] %s",
            currentActionIndex + 1,
            actions.size(),
            currentAction.getName());
    }

    /**
     * Returns whether the executor has started.
     *
     * @return true if started, false otherwise
     */
    public boolean isStarted() {
        return started;
    }
}
