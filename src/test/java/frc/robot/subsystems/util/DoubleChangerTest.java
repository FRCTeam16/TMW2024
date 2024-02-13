package frc.robot.subsystems.util;

import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;


public class DoubleChangerTest {

    @Test
    public void testChangeMethodValueChangesWhenDifferenceExceedsEpsilon() {
        DoubleChanger doubleChanger = new DoubleChanger();
        double oldValue = 1.0;
        double newValue = 1.0002;
        double changedValue = doubleChanger.change(oldValue, newValue);
        assertEquals(newValue, changedValue, doubleChanger.getEpsilon());
        assertTrue(doubleChanger.isChanged());
    }
    
    @Test
    public void testChangeMethodValueRemainsSameWhenDifferenceSmallerThanEpsilon() {
        DoubleChanger doubleChanger = new DoubleChanger();
        double oldValue = 1.0;
        double newValue = 1.000_000_1;
        double sameValue = doubleChanger.change(oldValue, newValue);
        assertEquals(oldValue, sameValue, doubleChanger.getEpsilon());
        assertFalse(doubleChanger.isChanged());
    }
    
    @Test
    public void testResetMethod() {
        DoubleChanger doubleChanger = new DoubleChanger();
        double oldValue = 1.0;
        double newValue = 1.0002;
        doubleChanger.change(oldValue, newValue);
        doubleChanger.reset();
        assertFalse(doubleChanger.isChanged());
    }
}