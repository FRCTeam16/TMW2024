package frc.robot.async;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.util.BSLogger;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestAsyncManager {

    @BeforeEach
    public void initialize() {
        // Initialize the FPGA timer
        BSLogger.log("TestAsyncManager", "Initializing: " + Timer.getFPGATimestamp());
    }

    @Test
    public void testUnregister() {
        AsyncManager asyncManager = new AsyncManager();
        asyncManager.register("test", () -> {
        });
        assert asyncManager.getNumberOfTasks() == 2;
        asyncManager.unregister("test");
        assert asyncManager.getNumberOfTasks() == 1;
    }

    @Test
    public void testExecution() {
        AsyncManager asyncManager = new AsyncManager();
        int[] counter = {0};
        asyncManager.register("test", () -> counter[0]++);
        asyncManager.start();
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            System.err.println("Interrupted exception");
        } finally {
            asyncManager.stop();
        }
        assert counter[0] > 0;
    }
}
