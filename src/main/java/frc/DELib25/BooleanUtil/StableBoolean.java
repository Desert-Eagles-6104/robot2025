package frc.DELib25.BooleanUtil;

import edu.wpi.first.wpilibj.Timer;

/**
 * A stable boolean that returns true only if the input has remained true
 * for at least the specified time threshold.
 */
public class StableBoolean {
    private double timeThreshold;
    private Timer timer;
    private boolean lastValue = false;

    public StableBoolean(double timeThreshold) {
        this.timeThreshold = timeThreshold;
        this.timer = new Timer();
        this.timer.start();
    }
    public void setTimeout(double timeThreshold) {
        this.timeThreshold = timeThreshold;
    }

    public boolean update(boolean value){
        if(!this.lastValue && value){
            this.timer.restart();
        }
        this.lastValue = value;
        return value && this.timer.hasElapsed(this.timeThreshold);
    }
}
