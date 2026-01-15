package utility.managers;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Executes a sequence of Steps.
 * Handles timing, conditionals, and looping automatically.
 *
 * Usage:
 * <pre>
 * StepExecutor executor = new StepExecutor();
 * executor.start(mySteps);
 *
 * // In your loop:
 * executor.update();
 * if (!executor.isRunning()) {
 *     // Sequence complete
 * }
 * </pre>
 */
public class StepExecutor {
    private Step[] steps;
    private int currentStep = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean running = false;

    /**
     * Start executing a sequence of steps.
     * @param sequence The steps to execute
     */
    public void start(Step[] sequence) {
        this.steps = sequence;
        this.currentStep = 0;
        this.running = true;
        timer.reset();
        executeCurrentStep();
    }

    /**
     * Stop execution immediately.
     */
    public void stop() {
        running = false;
    }

    /**
     * Call this every loop iteration to progress the sequence.
     */
    public void update() {
        if (!running || steps == null || currentStep >= steps.length) {
            running = false;
            return;
        }

        Step step = steps[currentStep];

        switch (step.type) {
            case ACTION:
                // Actions execute on start, so just advance
                advance();
                break;

            case WAIT:
                // Check time or condition
                boolean complete;
                if (step.condition != null) {
                    complete = step.condition.getAsBoolean();
                } else {
                    complete = timer.seconds() >= step.waitTime;
                }
                if (complete) {
                    advance();
                }
                break;

            case CONDITIONAL:
                // If condition true, skip forward; else just advance
                if (step.condition != null && step.condition.getAsBoolean()) {
                    currentStep += step.jumpSteps;
                }
                advance();
                break;

            case LOOP_END:
                // If condition true, jump back; else advance (exit loop)
                if (step.condition != null && step.condition.getAsBoolean()) {
                    currentStep -= step.jumpSteps;
                    timer.reset();
                    executeCurrentStep();
                } else {
                    advance();
                }
                break;

            case DONE:
                running = false;
                break;
        }
    }

    /**
     * Advance to the next step and execute its entry action.
     */
    private void advance() {
        currentStep++;
        timer.reset();
        if (currentStep < steps.length) {
            executeCurrentStep();
        } else {
            running = false;
        }
    }

    /**
     * Execute the current step's action (for ACTION type steps).
     */
    private void executeCurrentStep() {
        if (currentStep >= steps.length) {
            running = false;
            return;
        }

        Step step = steps[currentStep];
        if (step.type == Step.Type.ACTION && step.action != null) {
            step.action.run();
        }
    }

    /**
     * @return true if currently executing a sequence
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * @return The name of the current step (for telemetry)
     */
    public String getCurrentStepName() {
        if (!running || steps == null || currentStep >= steps.length) {
            return "Idle";
        }
        return steps[currentStep].name;
    }

    /**
     * @return Current step index (for debugging)
     */
    public int getCurrentStepIndex() {
        return currentStep;
    }

    /**
     * @return Total number of steps in current sequence
     */
    public int getTotalSteps() {
        return steps != null ? steps.length : 0;
    }
}
