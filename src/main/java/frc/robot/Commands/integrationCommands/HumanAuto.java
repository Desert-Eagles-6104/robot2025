package frc.robot.Commands.integrationCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.presetState.PresetState;
import frc.robot.Commands.IntakeCommands.IntakeForTimeAuto;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper2Subsystem;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class HumanAuto extends SequentialCommandGroup {
  /**  sequential command group to get the elevator and arm to the corect positions to be able to intake from human. */
  public HumanAuto(ElevatorSubsystem m_elevator , GripperArmSubsystem m_gripperArm, GripperSubsystem m_gripper ,PresetState m_state,Gripper2Subsystem m_gripper2) {
    addCommands(
      new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.Human),
      new WaitCommand(0.1),
      new IntakeForTimeAuto(m_gripper, 0.6, 0.7)
    );//TODO change to intake until beambreak sees
  }
}