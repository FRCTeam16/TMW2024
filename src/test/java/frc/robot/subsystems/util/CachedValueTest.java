package frc.robot.subsystems.util;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class CachedValueTest {

    private boolean integerComparer(Integer a, Integer b) {
        return !a.equals(b);
    }

    private boolean doubleComparer(Double a, Double b) {
        return DoubleChanger.areDifferent(a, b);
    }

    @Test
    public void testUpdate_SameValueAsInitial() {
        // Prepare data
        Integer initial = 1;
        CachedValue<Integer> cachedValue = new CachedValue<>(initial, this::integerComparer);

        // Test update method
        boolean result = cachedValue.update(initial);
        // Assert that update returns false if values are the same
        Assertions.assertFalse(result, "Update method should return false if values are the same");
    }

    @Test
    public void testUpdate_DifferentValueThanInitial() {
        // Prepare data
        Integer initial = 1;
        Integer newValue = 2;
        CachedValue<Integer> cachedValue = new CachedValue<>(initial, this::integerComparer);

        // Test update method
        boolean result = cachedValue.update(newValue);
        // Assert that update returns true if values are different
        Assertions.assertTrue(result, "Update method should return true if values are not the same");
    }

    @Test
    public void testUpdate_NullValue() {
        // Prepare data
        Integer initialValue = null;
        Integer newValue = 1;
        CachedValue<Integer> cachedValue = new CachedValue<>(initialValue, this::integerComparer);

        // Test update method
        boolean result = cachedValue.update(newValue);
        // Assert that update returns true if one value is null and the other is not
        Assertions.assertTrue(result, "Update method should return true if one value is null and the other is not");
    }

    @Test
    public void testUpdate_Double_SameValueAsInitial() {
        // Prepare data
        Double initial = 1.0;
        CachedValue<Double> cachedValue = new CachedValue<>(initial, this::doubleComparer);

        // Test update method
        boolean result = cachedValue.update(initial);
        // Assert that update returns false if values are the same
        Assertions.assertFalse(result, "Update method for Double should return false if values are the same");
    }

    @Test
    public void testUpdate_Double_DifferentValueThanInitial() {
        // Prepare data
        Double initial = 1.0;
        Double newValue = 2.0;
        CachedValue<Double> cachedValue = new CachedValue<>(initial, this::doubleComparer);

        // Test update method
        boolean result = cachedValue.update(newValue);
        // Assert that update returns true if values are different
        Assertions.assertTrue(result, "Update method for Double should return true if values are not the same");
    }

    @Test
    public void testUpdate_Double_NullValue() {
        // Prepare data
        Double initialValue = null;
        Double newValue = 1.0;
        CachedValue<Double> cachedValue = new CachedValue<>(initialValue, this::doubleComparer);

        // Test update method
        boolean result = cachedValue.update(newValue);
        // Assert that update returns true if one value is null and the other is not
        Assertions.assertTrue(result, "Update method for Double should return true if one value is null and the other is not");
    }
}