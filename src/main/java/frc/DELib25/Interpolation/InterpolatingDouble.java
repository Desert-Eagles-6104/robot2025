package frc.DELib25.Interpolation;

public final class InterpolatingDouble extends AbstractInterpolatingNumber<Double> {

    public InterpolatingDouble(Double value) {
        super(value);
    }

    @Override
    protected double toDouble() {
        return value.doubleValue();
    }

    @Override
    protected Double fromDouble(double d) {
        return d;
    }

    @Override
    protected InterpolatingDouble newInstance(Double v) {
        return new InterpolatingDouble(v);
    }
}