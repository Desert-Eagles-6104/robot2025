package frc.robot.Commands.GripperArmCommands;

import frc.DELib25.Subsystems.MotorSubsystems.Commands.MotorSubsystemSetMotionMagicPosition;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

/** Add your docs here. */
public class GripperArmSetPosition extends MotorSubsystemSetMotionMagicPosition {
    public GripperArmSetPosition(MotorSubsystemTalon ServoSubsystemTalon, double Position) {
        super(ServoSubsystemTalon, () -> Position);
    }
}
