package frc.robot.async;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.util.BSLogger;
import org.ejml.dense.row.CommonOps_FDRM;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReferenceArray;

public class AsyncManager {
    private final AsyncWorkThread thread = new AsyncWorkThread(0.005); // 5ms
    private final AtomicBoolean running = new AtomicBoolean(false);
    private final List<AsyncTask> tasks = Collections.synchronizedList(new ArrayList<>());

    public AsyncManager() {
        thread.setDaemon(true);
    }

    public void start() {
        BSLogger.log("AsyncManager", "Starting async tasks");
        running.set(true);
        thread.start();
    }

    public void stop() {
        BSLogger.log("AsyncManager", "Stopping async tasks");
        running.set(false);
    }

    public void register(String taskName, Runnable runnable) {
        tasks.add(new AsyncTask(taskName, runnable));
    }

    public void unregister(String taskName) {
        tasks.removeIf(task -> task.getName().equals(taskName));
    }

    public int getNumberOfTasks() {
        return tasks.size();
    }

    class AsyncWorkThread extends Thread {
        private final double period;
        private double lastRun = Timer.getFPGATimestamp();

        int counter = 0;

        AsyncWorkThread(double period) {
            this.period = period;
        }

        @Override
        public void run() {
            while (running.get()) {
//                if (counter++ % 1000 == 0) {
//                    BSLogger.log("AsyncWorkThread", "COUNTER: " + counter);
//                }
                if (Timer.getFPGATimestamp() - lastRun > period) {
                    synchronized (tasks) {
                        for (AsyncTask task : tasks) {
                            task.run();
                        }
                    }
                    double now = Timer.getFPGATimestamp();
                    if ((now - lastRun) > period * 2) {
                        BSLogger.log("AsyncManager", "Async tasks are running behind");
                    }
                    lastRun = Timer.getFPGATimestamp();
                }
            }
        }
    }
}
