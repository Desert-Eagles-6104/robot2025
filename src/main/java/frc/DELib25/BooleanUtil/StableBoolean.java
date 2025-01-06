package frc.DELib25.BooleanUtil;

import edu.wpi.first.wpilibj.Timer;

public class StableBoolean {
    private final double m_timeThreshold;
    private Timer m_timer;
    private boolean m_previousValue = false;

    public StableBoolean(double timeThreshold){
        m_timeThreshold = timeThreshold;
        m_timer = new Timer();
        m_timer.start();
    }

    public boolean get(boolean input){
        if(!m_previousValue && input){
            m_timer.reset();
        }
        m_previousValue = input;
        return input && m_timer.hasElapsed(m_timeThreshold);
    }

    public void reset(){
        m_timer.reset();
    }
}
