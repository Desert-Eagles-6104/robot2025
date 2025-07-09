package frc.DELib25.BooleanUtil;

/**
 * A pending boolean that returns a value after a specified minimum time has passed since the last change.
 * If the value changes before the time has passed, it will return the default value until the time is met.
 */
public class PendingBoolean {
    private final boolean defaultValue;
    private boolean lastValue;
    private final double minTime;
    private double lastChangeTime;

    public PendingBoolean(double minTime, boolean defaultValue) {
        this.minTime = minTime;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;
        this.lastChangeTime = Double.NaN;
    }

    public boolean update(boolean value, double timeStamp) {
        if (value != lastValue && Double.isNaN(lastChangeTime)) {
            lastChangeTime = timeStamp;
        }
        // If the time has passed return the value
        if (!Double.isNaN(this.lastChangeTime) && (timeStamp - this.lastChangeTime >= this.minTime)) {
            this.lastValue = value;
            this.lastChangeTime = Double.NaN;
        }
        return (Double.isNaN(this.lastChangeTime)) ? value : this.defaultValue;
    }
}
