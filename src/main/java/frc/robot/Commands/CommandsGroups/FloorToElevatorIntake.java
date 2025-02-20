// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandsGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Commands.GripperCommands.SmartGripperControl;
import frc.robot.Commands.IntakeCommands.ControlIntake;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.GripperSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FloorToElevatorIntake extends ParallelCommandGroup {
  /** Creates a new FloorToElevatorIntake. */
  private GripperSubsystem m_CoralIntakeSubsystem;
  private IntakeSubsystem m_AlgaeIntakeSubsystem;
  private double m_CoralPrecent = 0;
  private double m_algaePrecent = 0; 


  public FloorToElevatorIntake(GripperSubsystem coralIntakeSubsystem , IntakeSubsystem algaeIntakeSubsystem , double CoralPrecent , double algaePrecent) {
    m_CoralIntakeSubsystem = coralIntakeSubsystem;
    m_AlgaeIntakeSubsystem = algaeIntakeSubsystem;
    m_CoralPrecent = CoralPrecent;
    m_algaePrecent = algaePrecent;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SmartGripperControl(coralIntakeSubsystem, CoralPrecent) , new ControlIntake(algaeIntakeSubsystem, algaePrecent));
  }
}
