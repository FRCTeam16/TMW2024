package frc.robot.subsystems.util;

import java.util.function.BiPredicate;
import java.util.function.DoublePredicate;

public class CachedValue<T> {
    private final BiPredicate<T, T> comparer;
    private T value;

    public CachedValue(T initialValue, BiPredicate<T,T> comparer) {
        this.value = initialValue;
        this.comparer = comparer;
    }

    public boolean update(T newValue) {
        if (comparer.test(value, newValue)) {
            value = newValue;
            return true;
        }
        return false;
    }
}
