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
    private boolean pending;

    public PendingBoolean(double minTime, boolean defaultValue) {
        this.minTime = minTime;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;
        this.lastChangeTime = Double.NaN;
        this.pending = false;
    }
    
    public boolean update(boolean value, double timeStamp) {
        if (value == this.lastValue) {
            this.pending = false;
            return this.lastValue;
        }

        if (!this.pending) {
            this.pending = true;
            this.lastChangeTime = timeStamp;
            return this.defaultValue;
        }
        // The time has passed, the change gose into effect
        if (timeStamp - this.lastChangeTime >= this.minTime) {
            this.lastValue = value;
            this.pending = false;
            return this.lastValue;
        }

        return this.defaultValue;
    }

}
