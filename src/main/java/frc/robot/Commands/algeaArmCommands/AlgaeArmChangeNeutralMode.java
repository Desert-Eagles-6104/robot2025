// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.algeaArmCommands;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeArmChangeNeutralMode extends InstantCommand {
  private ElevatorSubsystem m_Elevator;
  private boolean hasChanged = false;
  public AlgaeArmChangeNeutralMode(ElevatorSubsystem elevator) {
    m_Elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(!hasChanged){
      m_Elevator.changeNeutralMode(NeutralModeValue.Coast);
      hasChanged = true;
    }
    else if(hasChanged){
      m_Elevator.changeNeutralMode(NeutralModeValue.Brake);
      hasChanged = false;
    }
  }
}