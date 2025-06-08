// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;
import frc.DELib25.Subsystems.ServoSubsystem.Commands.ServoSubsystemSetPosition;

/** Add your docs here. */
public class ElevatorSetPosition extends ServoSubsystemSetPosition {
    public ElevatorSetPosition(ServoSubsystemTalon ServoSubsystemTalon, double Position, boolean motionMagic) {
        super(ServoSubsystemTalon, Position, motionMagic);
    }
}
