package framework;

import java.util.ArrayList;
import java.util.List;

/**
 * Composite action that can run multiple actions either sequentially or in parallel.
 */
public class ActionGroup implements Action {
    public enum ExecutionMode {
        SEQUENTIAL,  // Run actions one after another
        PARALLEL     // Run all actions simultaneously until all complete
    }

    private final List<Action> actions;
    private final ExecutionMode mode;
    private int currentActionIndex;
    private boolean started;

    /**
     * Creates an action group with the specified execution mode.
     *
     * @param mode the execution mode (SEQUENTIAL or PARALLEL)
     */
    public ActionGroup(ExecutionMode mode) {
        this.actions = new ArrayList<>();
        this.mode = mode;
        this.currentActionIndex = 0;
        this.started = false;
    }

    /**
     * Adds an action to this group.
     *
     * @param action the action to add
     */
    public void addAction(Action action) {
        actions.add(action);
    }

    @Override
    public void start() {
        started = true;
        if (mode == ExecutionMode.SEQUENTIAL) {
            // Start only the first action
            if (!actions.isEmpty()) {
                actions.get(0).start();
            }
        } else {
            // PARALLEL: Start all actions at once
            for (Action action : actions) {
                action.start();
            }
        }
    }

    @Override
    public void update() {
        if (!started) {
            return;
        }

        if (mode == ExecutionMode.SEQUENTIAL) {
            // Update current action and advance if complete
            if (currentActionIndex < actions.size()) {
                Action currentAction = actions.get(currentActionIndex);
                currentAction.update();

                if (currentAction.isComplete()) {
                    currentAction.end();
                    currentActionIndex++;

                    // Start the next action if there is one
                    if (currentActionIndex < actions.size()) {
                        actions.get(currentActionIndex).start();
                    }
                }
            }
        } else {
            // PARALLEL: Update all actions
            for (Action action : actions) {
                if (!action.isComplete()) {
                    action.update();
                }
            }
        }
    }

    @Override
    public boolean isComplete() {
        if (!started || actions.isEmpty()) {
            return true;
        }

        if (mode == ExecutionMode.SEQUENTIAL) {
            // Complete when we've finished all actions
            return currentActionIndex >= actions.size();
        } else {
            // PARALLEL: Complete when all actions are complete
            for (Action action : actions) {
                if (!action.isComplete()) {
                    return false;
                }
            }
            return true;
        }
    }

    @Override
    public void end() {
        if (mode == ExecutionMode.PARALLEL) {
            // End all actions that haven't been ended yet
            for (Action action : actions) {
                if (action.isComplete()) {
                    action.end();
                }
            }
        }
        // For sequential mode, the last action's end() is already called in update()
    }

    @Override
    public String getName() {
        StringBuilder sb = new StringBuilder();
        sb.append(mode == ExecutionMode.SEQUENTIAL ? "Sequential[" : "Parallel[");
        for (int i = 0; i < actions.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(actions.get(i).getName());
        }
        sb.append("]");
        return sb.toString();
    }
}
