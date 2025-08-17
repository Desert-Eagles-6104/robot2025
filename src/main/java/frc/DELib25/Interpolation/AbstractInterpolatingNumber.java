package frc.DELib25.Interpolation;
import edu.wpi.first.math.interpolation.Interpolatable;
import java.util.Objects;

/**
 * Generic base for “interpolating numbers”.
 * Stores a value as T but does the math in double-space.
 *
 */
public abstract class AbstractInterpolatingNumber<T extends Number & Comparable<T>>
        implements Interpolatable<AbstractInterpolatingNumber<T>>,
                   InverseInterpolable<AbstractInterpolatingNumber<T>>,
                   Comparable<AbstractInterpolatingNumber<T>> {

    protected final T value;

    protected AbstractInterpolatingNumber(T value) {
        this.value = Objects.requireNonNull(value);
    }

    public T getValue() {
        return this.value;
    }

    protected abstract double toDouble();

    protected abstract T fromDouble(double d);

    @Override
    public AbstractInterpolatingNumber<T> interpolate(AbstractInterpolatingNumber<T> other, double x) {
        double y0 = this.toDouble();
        double y1 = other.toDouble();
        double y  = y0 + (y1 - y0) * x;
        return newInstance(fromDouble(y));
    }

    /**
     * Factory hook so the base class can return the right runtime type.
     * Each subclass should implement `newInstance(T v)` to return its own type.
     */
    protected abstract AbstractInterpolatingNumber<T> newInstance(T v);

    @Override
    public double inverseInterpolate(AbstractInterpolatingNumber<T> upper, AbstractInterpolatingNumber<T> query) {
        double lo = this.toDouble();
        double hi = upper.toDouble();
        double q  = query.toDouble();

        double range = hi - lo;
        if (range <= 0.0) return 0.0;

        double rel = q - lo;
        if (rel <= 0.0) return 0.0;

        return rel / range;
    }

    @Override
    public int compareTo(AbstractInterpolatingNumber<T> other) {
        return Double.compare(this.toDouble(), other.toDouble());
    }

    @Override
    public String toString() {
        return String.valueOf(value);
    }

    @Override
    public int hashCode() {
        return value.hashCode();
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof AbstractInterpolatingNumber<?> other)) return false;
        return Double.doubleToLongBits(this.toDouble()) == Double.doubleToLongBits(other.toDouble());
    }
}
