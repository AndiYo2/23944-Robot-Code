package utility.managers.actions;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/**
 * Repeats a group of actions while a condition is true.
 * When the condition becomes false, the action completes.
 */
public class RepeatWhileAction implements SpindexerAction {
    private final BooleanSupplier condition;
    private final List<SpindexerAction> actions;
    private int currentIndex;
    private boolean finished;

    public RepeatWhileAction(BooleanSupplier condition) {
        this.condition = condition;
        this.actions = new ArrayList<>();
        this.currentIndex = 0;
        this.finished = false;
    }

    public void addAction(SpindexerAction action) {
        actions.add(action);
    }

    @Override
    public void start() {
        currentIndex = 0;
        finished = false;

        // Check condition before starting
        if (!condition.getAsBoolean() || actions.isEmpty()) {
            finished = true;
            return;
        }

        actions.get(0).start();
    }

    @Override
    public void update() {
        if (finished || actions.isEmpty()) return;

        SpindexerAction current = actions.get(currentIndex);
        current.update();

        if (current.isComplete()) {
            current.end();
            currentIndex++;

            // If we've completed all actions in the loop
            if (currentIndex >= actions.size()) {
                // Check if we should repeat
                if (condition.getAsBoolean()) {
                    // Reset and start over
                    currentIndex = 0;
                    actions.get(0).start();
                } else {
                    // Condition is false, we're done
                    finished = true;
                }
            } else {
                // Move to next action in the loop
                actions.get(currentIndex).start();
            }
        }
    }

    @Override
    public boolean isComplete() {
        return finished;
    }

    @Override
    public void end() {}

    @Override
    public String getName() {
        if (finished || actions.isEmpty()) {
            return "RepeatWhile[Done]";
        }
        return "RepeatWhile[" + actions.get(currentIndex).getName() + "]";
    }
}
