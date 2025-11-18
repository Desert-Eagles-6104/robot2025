package frc.DELib25.BooleanUtil;

/**
 * A toggle boolean that flips its state on each rising edge of the input.
 */
public class ToggleBoolean {
    private boolean lastInput = false;
    private boolean toggleState = false;

    public boolean update(boolean input) {
        if (input && !this.lastInput) {
            this.toggleState = !this.toggleState;
        }
        this.lastInput = input;
        return this.toggleState;
    }
}
