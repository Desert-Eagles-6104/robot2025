package frc.DELib25.Interpolation;

public final class InterpolatingLong extends AbstractInterpolatingNumber<Long> {

    public InterpolatingLong(Long value) {
        super(value);
    }

    @Override
    protected double toDouble() {
        return value.doubleValue();
    }

    @Override
    protected Long fromDouble(double d) {
        return Math.round(d);
    }

    @Override
    protected InterpolatingLong newInstance(Long v) {
        return new InterpolatingLong(v);
    }
}