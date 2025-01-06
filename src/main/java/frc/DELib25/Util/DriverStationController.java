// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class DriverStationController extends GenericHID {
    final int m_port;
    public DriverStationController(int port){
        super(port);
        m_port = port;
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
      return new Trigger(CommandScheduler.getInstance().getDefaultButtonLoop(), () -> DriverStation.getStickButton(m_port, button));
    }

    /**
    * Constructs an event instance around the options button's digital signal.
    *
    * @param loop the event loop instance to attach the event to.
    * @return an event instance representing the options button's digital signal attached to the
    *     given loop.
    */
    public Trigger LeftBlue(){
      return button(Button.kLeftBlue.value);
    }

    /**
    * Constructs an event instance around the options button's digital signal.
    *
    * @param loop the event loop instance to attach the event to.
    * @return an event instance representing the options button's digital signal attached to the
    *     given loop.
    */
    public Trigger RightYellow(){
      return button(Button.kRightYellow.value);
    }

    /**
    * Constructs an event instance around the options button's digital signal.
    *
    * @param loop the event loop instance to attach the event to.
    * @return an event instance representing the options button's digital signal attached to the
    *     given loop.
    */
    public Trigger DownYellow(){
      return button(Button.kDownYellow.value);
    }

    /**
    * Constructs an event instance around the options button's digital signal.
    *
    * @param loop the event loop instance to attach the event to.
    * @return an event instance representing the options button's digital signal attached to the
    *     given loop.
    */
    public Trigger UpBlue(){
      return button(Button.kUpBlue.value);
    }

    /**
    * Constructs an event instance around the options button's digital signal.
    *
    * @param loop the event loop instance to attach the event to.
    * @return an event instance representing the options button's digital signal attached to the
    *     given loop.
    */
    public Trigger LeftSwitch(){
      return button(Button.kLeftSwitch.value);
    }

    /**
    * Constructs an event instance around the options button's digital signal.
    *
    * @param loop the event loop instance to attach the event to.
    * @return an event instance representing the options button's digital signal attached to the
    *     given loop.
    */
    public Trigger LeftMidSwitch(){
      return button(Button.kLeftMidSwitch.value);
    }

    /**
    * Constructs an event instance around the options button's digital signal.
    *
    * @param loop the event loop instance to attach the event to.
    * @return an event instance representing the options button's digital signal attached to the
    *     given loop.
    */
    public Trigger RightMidSwitch(){
      return button(Button.kRightMidSwitch.value);
    }

    /**
    * Constructs an event instance around the options button's digital signal.
    *
    * @param loop the event loop instance to attach the event to.
    * @return an event instance representing the options button's digital signal attached to the
    *     given loop.
    */
    public Trigger RightSwitch(){
      return button(Button.kRightSwitch.value);
    }

    /** Represents a digital button on a PS5Controller. */
    public enum Button {
        kLeftBlue(1),
        kRightYellow(2),
        kDownYellow(3),
        kUpBlue(4),
        kLeftSwitch(9),
        kLeftMidSwitch(10),
        kRightMidSwitch(11),
        kRightSwitch(12);

        /** Button value. */
        public final int value;

        Button(int index) {
        this.value = index;
        }
    }
}
