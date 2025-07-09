package frc.DELib25.BooleanUtil;

/**
 * An iterative boolean latch.
 * Returns true if the value of newValue changes last from false to true.
 */
public class IterativeLatchedBoolean {
    private boolean last = false;

    public boolean update(boolean newValue) {
    	boolean ret = false;
        if (newValue && !this.last) {
            ret = true;
        }
        this.last = newValue;
        return ret;
    }
}