package framework;

/**
 * Base interface for all autonomous actions.
 * Actions represent atomic operations in an autonomous routine (move, shoot, intake, etc.)
 * that can be sequenced or run in parallel.
 */
public interface Action {
    /**
     * Called once when the action is first started.
     * Use this to initialize any state or start operations.
     */
    void start();

    /**
     * Called repeatedly while the action is running.
     * Use this to update the action's state and progress.
     */
    void update();

    /**
     * Returns whether this action has completed.
     * When true, the executor will call end() and move to the next action.
     *
     * @return true if the action is complete, false otherwise
     */
    boolean isComplete();

    /**
     * Called once when the action completes.
     * Use this to clean up resources or perform final operations.
     */
    void end();

    /**
     * Returns a descriptive name for this action.
     * Useful for debugging and logging.
     *
     * @return the action's name
     */
    String getName();
}
