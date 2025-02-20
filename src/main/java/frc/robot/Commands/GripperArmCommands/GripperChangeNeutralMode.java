// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.GripperArmCommands;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GripperChangeNeutralMode extends InstantCommand {
  private GripperArmSubsystem m_CoralArmSubsystem;
  private boolean hasChanged = false;
  public GripperChangeNeutralMode(GripperArmSubsystem arm) {
    m_CoralArmSubsystem = arm;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!hasChanged){
      m_CoralArmSubsystem.changeNeutralMode(NeutralModeValue.Coast);
      hasChanged = true;
    }
    else if(hasChanged){
      m_CoralArmSubsystem.changeNeutralMode(NeutralModeValue.Brake);
      hasChanged = false;
    }
  }
}