// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.integrationCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class SmartPreset extends Command {
  private ElevatorSubsystem m_Elevator;
  private CoralArmSubsystem m_CoralArm;
  private AlgaeArmSubsystem m_AlgaeArm;
  private double m_CoralArmAngle = 0;
  private double m_AlgaeArmAngle = 0;
  private double m_ElevatorPosition = 0;

  /** Creates a new command to simply control set preset of the robot with all its subsystems. */
  public SmartPreset(ElevatorSubsystem elevator , CoralArmSubsystem coralArmSubsystem, AlgaeArmSubsystem algaeArmSubsystem ,double ElevatorPosition, double AlgeaArmAngle, double CoralArmAngle) {
    m_Elevator = elevator;
    m_CoralArm = coralArmSubsystem;
    m_AlgaeArm = algaeArmSubsystem;
    m_CoralArmAngle = CoralArmAngle;
    m_AlgaeArmAngle = AlgeaArmAngle;
    m_ElevatorPosition = ElevatorPosition;
    addRequirements(elevator, coralArmSubsystem, algaeArmSubsystem);
  }

  // public SmartPreset(ShooterSubsystem shooter , ArmSubsystem arm, double angle ,double velocity, boolean spin) {
  //   m_Elevator = shooter;
  //   m_arm = arm;
  //   m_angle = angle;
  //   m_velocity = velocity;
  //   m_spin = spin;
  //   addRequirements(arm , shooter);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_AlgaeArm.setMotionMagicPosition(m_AlgaeArmAngle);
    m_CoralArm.setMotionMagicPosition(m_CoralArmAngle);
    m_Elevator.setMotionMagicPosition(m_ElevatorPosition);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elevator.isAtSetpoint() && m_CoralArm.isAtSetpoint() && m_AlgaeArm.isAtSetpoint();
  }
}