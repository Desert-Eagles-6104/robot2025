package frc.DELib25.BooleanUtil;

/**
 * This boolean enforces a minimum time for the value to be true.  It captures a rising edge and enforces
 * based on timestamp.
 */
public class MinTimeBoolean {
    private IterativeLatchedBoolean latchedBoolean;
    private final double minTime;
    private double risingEdgeTime;

    public MinTimeBoolean(double minTime) {
        latchedBoolean = new IterativeLatchedBoolean();
        this.minTime = minTime;
        risingEdgeTime = Double.NaN;
    }

    public boolean update(boolean value, double timestamp) {
        if (latchedBoolean.update(value)) {
            risingEdgeTime = timestamp;
        }

        if (!value && !Double.isNaN(risingEdgeTime)
                && (timestamp - risingEdgeTime < minTime)) {
            return true;
        }
        return value;
    }
}
