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
    private final TimeDelayedBoolean isHeld = new TimeDelayedBoolean();

    public MultiTrigger(double timeout) {
        this.timeout = timeout;
    }

    public void update(boolean pressed) {
        this.lastPressed = pressed;
        this.lastTapped = this.wasTapped.update(pressed);
        this.isHeld.update(pressed, this.timeout);
    }

    public boolean wasTapped() {
        return this.lastTapped;
    }

    public boolean isPressed() {
        return this.lastPressed;
    }

    public boolean isHeld() {
        return this.isHeld.update(this.lastPressed, this.timeout);
    }

    public boolean holdStarted() {
        return this.wasHeld.update(this.isHeld());
    }
}
