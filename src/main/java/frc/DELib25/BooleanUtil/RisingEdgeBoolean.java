package frc.DELib25.BooleanUtil;

/**
 * An rising edge boolean latch.
 * Returns true if the value of newValue changes last from false to true.
 */
public class RisingEdgeBoolean {
    private boolean lastValue = false;

    public boolean update(boolean newValue) {
    	boolean ret = false;
        if (newValue && !this.lastValue) {
            ret = true;
        }
        this.lastValue = newValue;
        return ret;
    }
}