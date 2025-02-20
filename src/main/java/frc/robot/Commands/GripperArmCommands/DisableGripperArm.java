// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.GripperArmCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GripperArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DisableGripperArm extends InstantCommand {
  GripperArmSubsystem m_coralArm;
  public DisableGripperArm(GripperArmSubsystem CoralArm) {
    m_coralArm = CoralArm;
    addRequirements(CoralArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralArm.disableMotors();
  }
}
