// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.GripperArmCommands;

import frc.DELib25.Subsystems.MotorSubsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;
import frc.DELib25.Subsystems.MotorSubsystems.ServoSubsystem.Commands.ServoSubsystemSetPosition;

/** Add your docs here. */
public class GripperArmSetPosition extends ServoSubsystemSetPosition {
    public GripperArmSetPosition(ServoSubsystemTalon ServoSubsystemTalon, double Position, boolean motionMagic) {
        super(ServoSubsystemTalon, Position, motionMagic);
    }}
