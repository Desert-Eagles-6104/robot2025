package frc.DELib25.BooleanUtil;

/**
 * Class to differentiate between tapping and holding a joystick button/trigger
 */
public class MultiTrigger {
    private final double timeout;
    private boolean lastPressed = false;
    private boolean lastTapped = false;
    private final IterativeLatchedBoolean wasTapped = new IterativeLatchedBoolean();
    private final IterativeLatchedBoolean wasHeld = new IterativeLatchedBoolean();
    private final StableBoolean isHeld;

    public MultiTrigger(double timeout) {
        this.timeout = timeout;
        this.isHeld = new StableBoolean(timeout);
        this.isHeld.setTimeout(this.timeout);
    }

    public void update(boolean pressed) {
        this.lastPressed = pressed;
        this.lastTapped = this.wasTapped.update(pressed);
        this.isHeld.update(pressed);
    }

    public boolean wasTapped() {
        return this.lastTapped;
    }

    public boolean isPressed() {
        return this.lastPressed;
    }

    public boolean isHeld() {
        return this.isHeld.update(this.lastPressed);
    }

    public boolean holdStarted() {
        return this.wasHeld.update(this.isHeld());
    }
}
