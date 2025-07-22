package frc.robot.Commands.integrationCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.DELib25.Subsystems.MotorSubsystems.Commands.MotorSubsystemDisableMotors;
import frc.robot.Commands.GripperCommands.GripperSet;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Gripper2Subsystem;
import frc.robot.subsystems.GripperArmSubsystem;

public class ResetSubsystems extends ParallelCommandGroup {
  /** Creates a new ResetSubsystems. */
  public ResetSubsystems(ElevatorSubsystem m_elevator , GripperArmSubsystem m_gripperArm, Gripper2Subsystem m_gripper) {
    addCommands(new GripperSet(m_gripper, 0), new MotorSubsystemDisableMotors(m_gripperArm));
  }
}
