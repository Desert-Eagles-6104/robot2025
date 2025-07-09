package frc.DELib25.BooleanUtil;

/**
 * A latched boolean.
 * Once set to true, it remains so until reset.
 */
public class LatchedBolean {
    private boolean isLatched = false;

    public void update(boolean newValue) {
        if (newValue) {
            this.isLatched = true;
        }
    }

    public void reset(){
        this.isLatched = false;
    }

    public boolean get(){
        return this.isLatched;
    }
}
