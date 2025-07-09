package frc.DELib25.BooleanUtil;

import edu.wpi.first.wpilibj.Timer;

/**
 * A stable boolean that returns true only if the input has remained true
 * for at least the specified time threshold.
 */
public class StableBoolean {
    private final double timeThreshold;
    private Timer timer;
    private boolean previousValue = false;

    public StableBoolean(double timeThreshold){
        this.timeThreshold = timeThreshold;
        this.timer = new Timer();
        this.timer.start();
    }

    public boolean get(boolean input){
        if(!this.previousValue && input){
            this.timer.reset();
        }
        this.previousValue = input;
        return input && this.timer.hasElapsed(this.timeThreshold);
    }

    public void reset(){
        this.timer.reset();
    }
}
