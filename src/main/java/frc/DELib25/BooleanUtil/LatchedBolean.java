package frc.DELib25.BooleanUtil;

/**
 * A latched boolean.
 * Once set to true, it remains so until reset.
 */
public class LatchedBolean {
    private boolean isLatched = false;

    public boolean update(boolean newValue) {
        this.isLatched |= newValue;
        return this.isLatched;
    }

    public void reset(){
        this.isLatched = false;
    }

    public boolean get(){
        return this.isLatched;
    }
}
