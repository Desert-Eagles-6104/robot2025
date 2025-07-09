package frc.DELib25.BooleanUtil;

/**
 * An iterative boolean latch.
 * <p>
 * Returns true once if and only if the value of newValue changes from false to true.
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