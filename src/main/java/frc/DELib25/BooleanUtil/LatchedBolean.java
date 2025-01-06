package frc.DELib25.BooleanUtil;

public class LatchedBolean {
    private boolean ret = false;

    public void update(boolean newValue) {
        if (newValue) {
            ret = true;
        }
    }

    public void reset(){
        ret = false;
    }

    public boolean get(){
        return ret;
    }
}
