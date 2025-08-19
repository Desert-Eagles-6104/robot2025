package frc.robot.Commands.integrationCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.presetState.PresetState;
import frc.DELib25.Subsystems.MotorSubsystems.Commands.MotorSubsystemDisableMotors;
import frc.robot.Commands.IntakeCommands.IntakeForTime;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper2Subsystem;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class L4Score extends SequentialCommandGroup {

  public L4Score(ElevatorSubsystem m_elevator , GripperArmSubsystem m_gripperArm, GripperSubsystem m_gripper ,PresetState m_state,Gripper2Subsystem m_gripper2, BooleanSupplier approve) {
    addCommands(
      new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.L4),
      new WaitCommand(0.3),
      new IntakeForTime(m_gripper, -0.6, Constants.Intake.TimeToDropIntegraion, approve),
      new WaitCommand(0.3),
      new SmartPreset(m_elevator, m_gripperArm, m_gripper, m_state.ZERO),
      new MotorSubsystemDisableMotors(m_gripperArm)
    );
  }
}