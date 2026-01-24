package utility.managers.actions;

/**
 * Executes a runnable immediately and completes.
 * Useful for custom one-off operations.
 */
public class RunnableAction implements SpindexerAction {
    private final String name;
    private final Runnable runnable;

    public RunnableAction(String name, Runnable runnable) {
        this.name = name;
        this.runnable = runnable;
    }

    @Override
    public void start() {
        runnable.run();
    }

    @Override
    public void update() {}

    @Override
    public boolean isComplete() {
        return true;
    }

    @Override
    public void end() {}

    @Override
    public String getName() {
        return name;
    }
}
