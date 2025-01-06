package frc.DELib25.BooleanUtil;

/**
 * An iterative boolean latch.
 * <p>
 * Returns true once if and only if the value of newValue changes from false to true.
 */
public class IterativeLatchedBoolean {
    private boolean mLast = false;

    public boolean update(boolean newValue) {
        boolean ret = false;
        if (newValue && !mLast) {
            ret = true;
        }
        mLast = newValue;
        return ret;
    }
}