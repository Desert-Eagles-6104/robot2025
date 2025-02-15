// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandsGroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.CoralIntakeCommands.ControlCoralintake;
import frc.robot.Commands.algaeIntakeCommands.ControlAlgaeIntake;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FloorToElevatorIntake extends ParallelCommandGroup {
  /** Creates a new FloorToElevatorIntake. */
  private CoralIntakeSubsystem m_CoralIntakeSubsystem;
  private AlgaeIntakeSubsystem m_AlgaeIntakeSubsystem;
  private double m_CoralPrecent = 0;
  private double m_algaePrecent = 0; 

  public FloorToElevatorIntake(CoralIntakeSubsystem coralIntakeSubsystem , AlgaeIntakeSubsystem algaeIntakeSubsystem , double CoralPrecent , double algaePrecent) {
    m_CoralIntakeSubsystem = coralIntakeSubsystem;
    m_AlgaeIntakeSubsystem = algaeIntakeSubsystem;
    m_CoralPrecent = CoralPrecent;
    m_algaePrecent = algaePrecent;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ControlCoralintake(coralIntakeSubsystem, CoralPrecent) , new ControlAlgaeIntake(algaeIntakeSubsystem, algaePrecent));
  }
}
