package frc.DELib25.BooleanUtil;

import edu.wpi.first.wpilibj.Timer;

public class StableBoolean {
    private final double timeThreshold;
    private Timer timer;
    private boolean previousValue = false;

    public StableBoolean(double timeThreshold){
        this.timeThreshold = timeThreshold;
        timer = new Timer();
        timer.start();
    }

    public boolean get(boolean input){
        if(!previousValue && input){
            timer.reset();
        }
        previousValue = input;
        return input && timer.hasElapsed(timeThreshold);
    }

    public void reset(){
        timer.reset();
    }
}
