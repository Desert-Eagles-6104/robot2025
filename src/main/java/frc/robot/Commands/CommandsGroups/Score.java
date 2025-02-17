// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandsGroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Commands.CoralIntakeCommands.SmartCoralControl;
import frc.robot.Commands.integrationCommands.SmartPreset;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score extends SequentialCommandGroup {
  private ElevatorSubsystem m_elevator;
  private CoralArmSubsystem m_coralArm;
  private AlgaeArmSubsystem m_AlgaeArm;
  private CoralIntakeSubsystem m_CoralIntakeSubsystem;
  private AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem;
  private double m_elevatorPosition = 0;
  private double m_CoralArmAngle = 0;
  private double m_AlgaeArmAngle = 0;
  private double m_CoralPrecent = 0;
  private double m_algaePrecent = 0; 
  
  /** Creates a new Intake. */
  public Score(ElevatorSubsystem elevator , CoralArmSubsystem coralArm, AlgaeArmSubsystem algaeArm , double ElevatorPosition , double AlgaeArmAngle , double CoralArmAngle, CoralIntakeSubsystem coralIntakeSubsystem , AlgaeIntakeSubsystem algaeIntakeSubsystem , double CoralPrecent , double algaePrecent) {
    m_elevator = elevator;
    m_coralArm = coralArm;
    m_AlgaeArm = algaeArm;
    m_elevatorPosition = ElevatorPosition;
    m_AlgaeArmAngle = AlgaeArmAngle;
    m_CoralArmAngle = CoralArmAngle;
    m_CoralIntakeSubsystem = coralIntakeSubsystem;
    m_AlgaeIntakeSubsystem = algaeIntakeSubsystem;
    m_CoralPrecent = CoralPrecent;
    m_algaePrecent = algaePrecent;

    addCommands(new SmartPreset(elevator, coralArm, algaeArm ,ElevatorPosition , AlgaeArmAngle , CoralArmAngle) , new SmartCoralControl(coralIntakeSubsystem, CoralPrecent), new SmartPreset( elevator, coralArm, algaeArm ,ElevatorPosition , AlgaeArmAngle , CoralArmAngle));
  }
}