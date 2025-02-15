// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.integrationCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Commands.CoralArmCommands.CoralArmHoming;
import frc.robot.Commands.ElevatorCommands.ElevatorHoming;
import frc.robot.Commands.algeaArmCommands.AlgaeArmHoming;
import frc.robot.Constants.AlgaeArm;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetAllSubsystems extends ParallelCommandGroup {
  /** Creates a new ResetAllSubsystems. */
  public ResetAllSubsystems(ElevatorSubsystem elevatorSubsystem, CoralArmSubsystem coralArmSubsystem, AlgaeArmSubsystem algaeArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ElevatorHoming(elevatorSubsystem), new CoralArmHoming(coralArmSubsystem), new AlgaeArmHoming(algaeArm));
  }
}
