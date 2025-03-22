// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.integrationCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GripperArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetAllSubsystems extends Command {
  ElevatorSubsystem m_elevator;
  GripperArmSubsystem m_gripperArm;

  /** Creates a new ResetAllSubsystems. */
  public ResetAllSubsystems(ElevatorSubsystem Elevator , GripperArmSubsystem gripperArm) {
    m_elevator = Elevator;
    m_gripperArm = gripperArm;
    addRequirements(m_elevator , m_gripperArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.ControlSoftLimit(false);
    m_gripperArm.ControlSoftLimit(false);

    m_elevator.setMotionMagicPosition(0.0);
    m_gripperArm.setMotionMagicPosition(-79.384765625);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.setPrecentOutput(-0.02);
    m_gripperArm.setPrecentOutput(-0.06);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.resetPosition(0);
    m_gripperArm.resetPosition(0);
    m_elevator.setPrecentOutput(0);
    m_gripperArm.setPrecentOutput(0);
    m_elevator.ControlSoftLimit(true);
    m_gripperArm.ControlSoftLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.getMagnetState() == true && m_gripperArm.isAtSetpoint();
  }
}