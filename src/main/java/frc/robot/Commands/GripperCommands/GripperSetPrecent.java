// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands.GripperCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.GripperSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class GripperSetPrecent extends Command {
//   GripperSubsystem m_CoralIntakeSubsystem;
//   private double m_output = 0;

//   /** Creates a new Controlintake. */
//   public GripperSetPrecent(GripperSubsystem coralIntakeSubsystem , double output) {
//     m_CoralIntakeSubsystem = coralIntakeSubsystem;
//     double m_output = output;
//     addRequirements(m_CoralIntakeSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_CoralIntakeSubsystem.setMotorPercent(m_output);
//   }
// }
