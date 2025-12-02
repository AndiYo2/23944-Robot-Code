package tests;
import java.util.Timer;
import java.util.TimerTask;

public class TestingTimerStuff {

    public static void main(String[] args) {
        // Create a new Timer object
        Timer timer = new Timer();

        // Define the TimerTask
        TimerTask task = new TimerTask() {
            @Override
            public void run() {
                System.out.println("This task ran after a 5-second delay!");
                // Optionally, cancel the timer if this is a one-time task
                timer.cancel();
            }
        };

        TimerTask timTask = new TimerTask() {
            @Override
            public void run() {
                System.out.println("This task ran after a 5-second delay too!");
            }
        };

        // Schedule the task to run after a 5-second (5000 milliseconds) delay
        long delayMillis = 5000;
        timer.schedule(task, delayMillis);
        timer.schedule(timTask, delayMillis);

        System.out.println("Task scheduled to run in 5 seconds.");
    }
}

