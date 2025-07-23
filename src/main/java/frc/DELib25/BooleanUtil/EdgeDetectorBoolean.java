package frc.DELib25.BooleanUtil;

/**
 * A utility class for detecting rising and falling edges of a boolean signal.
 */
public class EdgeDetectorBoolean {
    private boolean lastValue = false;

    /**
     * Updates the internal state and returns true if a rising edge is detected.
     */
    public boolean risingEdge(boolean newValue) {
        boolean result = !lastValue && newValue;
        lastValue = newValue;
        return result;
    }

    /**
     * Updates the internal state and returns true if a falling edge is detected.
     */
    public boolean fallingEdge(boolean newValue) {
        boolean result = lastValue && !newValue;
        lastValue = newValue;
        return result;
    }

    public boolean getLastValue() {
        return lastValue;
    }

}