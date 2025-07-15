// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.GripperCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper2Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GripperSetForAuto extends Command {
  Gripper2Subsystem m_gripper;
  double m_output;
  private Timer m_Timer;
  /** Creates a new GripperSet. */
  public GripperSetForAuto(Gripper2Subsystem gripper , double output ) {
    m_gripper = gripper;
    m_output = output;
    addRequirements(m_gripper);
    m_Timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Grip");
    m_Timer.reset();
    m_gripper.setPercent(m_output);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   // if(m_Timer.hasElapsed(11)&& !m_Timer.hasElapsed(15)){
      // m_gripper.disableMotors();
     // }
     // else{
        m_gripper.setPercent(m_output);
      //}
      // done = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.disableMotors();
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
