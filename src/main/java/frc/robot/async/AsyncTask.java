package frc.robot.async;

class AsyncTask {
    private final String name;
    private final Runnable runnable;

    public AsyncTask(String name, Runnable runnable) {
        this.name = name;
        this.runnable = runnable;
    }

    public void run() {
        runnable.run();
    }
    public String getName() {
        return name;
    }

}
