// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.integrationCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.GripperArmCommands.DisableGripperArm;
import frc.robot.Commands.GripperCommands.GripperSet;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper2Subsystem;
import frc.robot.subsystems.GripperArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetSubsystems extends ParallelCommandGroup {
  /** Creates a new ResetSubsystems. */
  public ResetSubsystems(ElevatorSubsystem m_elevator , GripperArmSubsystem m_gripperArm, Gripper2Subsystem m_gripper) {
    addCommands(new GripperSet(m_gripper, 0), new DisableGripperArm(m_gripperArm));
  }
}
