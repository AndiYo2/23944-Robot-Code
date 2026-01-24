package utility.managers.actions;

/**
 * Base interface for spindexer sequence actions.
 * Matches the pattern used by framework.Action.
 */
public interface SpindexerAction {
    void start();
    void update();
    boolean isComplete();
    void end();
    String getName();
}
