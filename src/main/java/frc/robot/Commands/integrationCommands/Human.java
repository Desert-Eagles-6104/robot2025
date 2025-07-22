package frc.robot.Commands.integrationCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.presetState.PresetState;
import frc.robot.Commands.IntakeCommands.EatUntilCoral;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper2Subsystem;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class Human extends SequentialCommandGroup {
  /**  sequential command group to get the elevator and arm to the corect positions to be able to intake from human. */
  public Human(ElevatorSubsystem m_elevator , GripperArmSubsystem m_gripperArm, GripperSubsystem m_gripper ,PresetState m_state,Gripper2Subsystem m_gripper2) {
    addCommands(
      new SmartPreset(m_elevator, m_gripperArm, m_gripper, PresetState.Human),
      new WaitCommand(0.1),
      new EatUntilCoral(m_gripper2)
    );//TODO change to intake until beambreak sees
  }
}