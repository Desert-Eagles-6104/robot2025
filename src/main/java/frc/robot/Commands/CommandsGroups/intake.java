// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandsGroups;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Commands.integrationCommands.SmartPreset;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class intake extends SequentialCommandGroup {
  private ElevatorSubsystem m_elevator;
  private GripperArmSubsystem m_coralArm;
  private IntakeArmSubsystem m_AlgaeArm;
  private GripperSubsystem m_CoralIntakeSubsystem;
  private IntakeSubsystem m_AlgaeIntakeSubsystem;
  private CommandPS5Controller m_driverController;
  private double m_elevatorPosition = 0;
  private double m_CoralArmAngle = 0;
  private double m_AlgaeArmAngle = 0;
  private double m_CoralPrecent = 0;
  private double m_algaePrecent = 0; 
  
  /** Creates a new Intake. */
  public intake(ElevatorSubsystem elevator , GripperArmSubsystem coralArm, IntakeArmSubsystem algaeArm , double ElevatorPosition , double AlgaeArmAngle , double CoralArmAngle, GripperSubsystem coralIntakeSubsystem , IntakeSubsystem algaeIntakeSubsystem , double CoralPrecent , double algaePrecent) {
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

    addCommands(new SmartPreset(elevator, coralArm, algaeArm ,ElevatorPosition , AlgaeArmAngle , CoralArmAngle) , new FloorToElevatorIntake(coralIntakeSubsystem, algaeIntakeSubsystem, CoralPrecent, algaePrecent) ,  new SmartPreset( elevator, coralArm, algaeArm ,ElevatorPosition , AlgaeArmAngle , CoralArmAngle));
  }
}