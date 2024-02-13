package frc.robot.subsystems.util;

import java.util.function.BiPredicate;

public class CachedValue<T> {
    private final BiPredicate<T, T> comparer;
    private T value;

    public CachedValue(T initialValue, BiPredicate<T,T> comparer) {
        this.value = initialValue;
        this.comparer = comparer;
    }

    public boolean update(T newValue) {
        if(value == null && newValue == null) {
            return false;
        }

        if(value == null || newValue == null){
            value = newValue;
            return true;
        }

        if (comparer.test(value, newValue)) {
            value = newValue;
            return true;
        }
        return false;
    }
}
