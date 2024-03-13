package frc.robot.async;

import org.junit.jupiter.api.Test;

public class TestAsyncManager {

    @Test
    public void testUnregister() {
        AsyncManager asyncManager = new AsyncManager();
        asyncManager.register("test", () -> {});
        assert asyncManager.getNumberOfTasks() == 1;
        asyncManager.unregister("test");
        assert asyncManager.getNumberOfTasks() == 0;
    }

    @Test
    public void testExecution() {
        AsyncManager asyncManager = new AsyncManager();
        int[] counter = {0};
        asyncManager.register("test", () -> counter[0]++);
        asyncManager.start();
        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            // expected
//            e.printStackTrace();
        } finally {
            asyncManager.stop();
        }
        assert counter[0] > 0;
    }
}
