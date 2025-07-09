package frc.DELib25.BooleanUtil;


/*  
    Read a provided input
    When the input becomes true, output true until manually cleared
    Useful for latching
*/
public class StickyBoolean {

    private boolean on = false;

    public StickyBoolean() {
        super();
        on = false;
    }

    public boolean update(boolean useVision) {
        on |= useVision;
        return on;
    }

    public void reset() {
        on = false;
    }

    public boolean get() {
        return on;
    }
}
