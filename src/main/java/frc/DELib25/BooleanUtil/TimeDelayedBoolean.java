package frc.DELib25.BooleanUtil;

import edu.wpi.first.wpilibj.Timer;

/** 
 * A time-delayed boolean that returns true if the value is true for a specified duration.
 */
public class TimeDelayedBoolean {
    private Timer timer = new Timer();
    private boolean last = false;

    public boolean update(boolean value, double timeout) {
        if (!this.last && value) {
            this.timer.reset();
            this.timer.start();
        }
        this.last = value;
        return value && this.timer.get() >= timeout;
    }
}
