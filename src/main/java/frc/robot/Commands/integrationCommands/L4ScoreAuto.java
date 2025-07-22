package frc.robot.Commands.integrationCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.DELib25.Subsystems.MotorSubsystems.Commands.MotorSubsystemDisableMotors;
import frc.DELib25.Subsystems.MotorSubsystems.Commands.MotorSubsystemSetMotionMagicPosition;
import frc.robot.Constants;
import frc.robot.presetState.PresetState;
import frc.robot.Commands.IntakeCommands.IntakeForTime;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper2Subsystem;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;

public class L4ScoreAuto extends SequentialCommandGroup {

  public L4ScoreAuto(ElevatorSubsystem m_elevator, GripperArmSubsystem m_gripperArm, GripperSubsystem m_gripper, PresetState m_state, Gripper2Subsystem m_gripper2, BooleanSupplier approve) {
    addCommands(
      new SmartPreset(m_elevator, m_gripperArm, m_gripper, PresetState.L4),
      new WaitCommand(0.9),
      new IntakeForTime(m_gripper, -0.6, Constants.Intake.TimeToDropIntegraion, approve),
      new WaitCommand(0.3),
      new MotorSubsystemSetMotionMagicPosition(m_gripperArm, () -> -88.0),
      new SmartPreset(m_elevator, m_gripperArm, m_gripper, PresetState.ZERO),
      new MotorSubsystemDisableMotors(m_gripperArm)
    );
  }
  
}