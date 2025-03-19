// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.integrationCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.presetState.PresetState;
import frc.robot.Commands.IntakeCommands.IntakeForTime;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper2Subsystem;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L2Score extends SequentialCommandGroup {
  /** Creates a new L2. */
  public L2Score(ElevatorSubsystem m_elevator , GripperArmSubsystem m_gripperArm, GripperSubsystem m_gripper ,PresetState m_state,Gripper2Subsystem m_gripper2) {
    addCommands((new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.L3)),(new WaitCommand(0.1)),(new IntakeForTime(m_gripper, 0.6, 0.3)),(new WaitCommand(0.3)),new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.ZERO),(new ResetSubsystems(m_elevator, m_gripperArm, m_gripper2)));
  }
}
