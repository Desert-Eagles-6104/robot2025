package frc.DELib25.Interpolation;

import edu.wpi.first.math.interpolation.Interpolatable;
import java.util.Objects;

/**
 * Self-typed generic base for interpolating numeric wrappers.
 *
 * S = the concrete subclass type (e.g., InterpolatingDouble)
 * T = the underlying number type (e.g., Double)
 */
public abstract class AbstractInterpolatingNumber<
        S extends AbstractInterpolatingNumber<S,T>,
        T extends Number & Comparable<T>>
    implements Interpolatable<S>, InverseInterpolable<S>, Comparable<S> {

  protected final T value;

  protected AbstractInterpolatingNumber(T value) {
    this.value = Objects.requireNonNull(value);
  }

  public T getValue() { return value; }

  protected abstract double toDouble();
  protected abstract T fromDouble(double d);
  protected abstract S newInstance(T v);

  @Override
  public S interpolate(S other, double x) {
    double y0 = this.toDouble();
    double y1 = other.toDouble();
    double y  = y0 + (y1 - y0) * x;
    return newInstance(fromDouble(y));
  }

  @Override
  public double inverseInterpolate(S upper, S query) {
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
  public int compareTo(S other) {
    return Double.compare(this.toDouble(), other.toDouble());
  }

  @Override
  public String toString() { return String.valueOf(value); }

  @Override
  public int hashCode() { return Double.hashCode(toDouble()); }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (!(o instanceof AbstractInterpolatingNumber<?,?> a)) return false;
    return Double.doubleToLongBits(this.toDouble()) == Double.doubleToLongBits(a.toDouble());
  }
}