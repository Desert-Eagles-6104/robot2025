// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.algeaArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DisableAlgaeArm extends InstantCommand {
  AlgaeArmSubsystem m_AlgaeArmSubsystem;
  public DisableAlgaeArm(AlgaeArmSubsystem arm) {
    m_AlgaeArmSubsystem = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_AlgaeArmSubsystem.disableMotors();
  }
}
