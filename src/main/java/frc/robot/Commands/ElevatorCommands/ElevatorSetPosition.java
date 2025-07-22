package frc.robot.Commands.ElevatorCommands;

import java.util.function.DoubleSupplier;

import frc.DELib25.Subsystems.MotorSubsystems.Commands.MotorSubsystemSetPosition;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

/** Add your docs here. */
public class ElevatorSetPosition extends MotorSubsystemSetPosition {
    public ElevatorSetPosition(MotorSubsystemTalon ServoSubsystemTalon, DoubleSupplier Position) {
        super(ServoSubsystemTalon, Position);
    }
}
