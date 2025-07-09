package frc.DELib25.BooleanUtil;


/** 
 * A sticky boolean that retains its true state once set to true.
 * It will remain so until reset.
*/
public class StickyBoolean {
    private boolean on = false;

    public boolean update(boolean useVision) {
        on |= useVision;
        return on;
    }

    public void reset() {
        on = false;
    }

    public boolean get() {
        return on;
    }
}
