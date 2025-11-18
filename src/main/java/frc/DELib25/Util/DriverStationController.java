package frc.DELib25.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DriverStationController extends GenericHID {
	final int port;
	public DriverStationController(int port){
		super(port);
		this.port = port;
	}

	/**
	 * Constructs an event instance around this button's digital signal.
	 *
	 * @param button the button index
	 * @return an event instance representing the button's digital signal attached to the {@link
	 *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
	 * @see #button(int, EventLoop)
	 */
	public Trigger button(int button) {
		return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> DriverStation.getStickButton(port, button));
	}

	/**
	* Constructs an event instance around the options button's digital signal.
	*
	* @param loop the event loop instance to attach the event to.
	* @return an event instance representing the options button's digital signal attached to the
	*     given loop.
	*/
	public Trigger LeftBlue(){
		return button(Button.LeftBlue.value);
	}

	/**
	* Constructs an event instance around the options button's digital signal.
	*
	* @param loop the event loop instance to attach the event to.
	* @return an event instance representing the options button's digital signal attached to the
	*     given loop.
	*/
	public Trigger RightYellow(){
		return button(Button.RightYellow.value);
	}

	/**
	* Constructs an event instance around the options button's digital signal.
	*
	* @param loop the event loop instance to attach the event to.
	* @return an event instance representing the options button's digital signal attached to the
	*     given loop.
	*/
	public Trigger DownYellow(){
		return button(Button.DownYellow.value);
	}

	/**
	* Constructs an event instance around the options button's digital signal.
	*
	* @param loop the event loop instance to attach the event to.
	* @return an event instance representing the options button's digital signal attached to the
	*     given loop.
	*/
	public Trigger UpBlue(){
		return button(Button.UpBlue.value);
	}

	/**
	* Constructs an event instance around the options button's digital signal.
	*
	* @param loop the event loop instance to attach the event to.
	* @return an event instance representing the options button's digital signal attached to the
	*     given loop.
	*/
	public Trigger LeftSwitch(){
		return button(Button.LeftSwitch.value);
	}

	/**
	* Constructs an event instance around the options button's digital signal.
	*
	* @param loop the event loop instance to attach the event to.
	* @return an event instance representing the options button's digital signal attached to the
	*     given loop.
	*/
	public Trigger LeftMidSwitch(){
		return button(Button.LeftMidSwitch.value);
	}

	/**
	* Constructs an event instance around the options button's digital signal.
	*
	* @param loop the event loop instance to attach the event to.
	* @return an event instance representing the options button's digital signal attached to the
	*     given loop.
	*/
	public Trigger RightMidSwitch(){
		return button(Button.RightMidSwitch.value);
	}

	/**
	* Constructs an event instance around the options button's digital signal.
	*
	* @param loop the event loop instance to attach the event to.
	* @return an event instance representing the options button's digital signal attached to the
	*     given loop.
	*/
	public Trigger RightSwitch(){
		return button(Button.RightSwitch.value);
	}

	/** Represents a digital button on a PS5Controller. */
	public enum Button {
		LeftBlue(1),
		RightYellow(2),
		DownYellow(3),
		UpBlue(4),
		LeftSwitch(9),
		LeftMidSwitch(10),
		RightMidSwitch(11),
		RightSwitch(12);

		/** Button value. */
		public final int value;

		Button(int index) {
			this.value = index;
		}
	}
}
