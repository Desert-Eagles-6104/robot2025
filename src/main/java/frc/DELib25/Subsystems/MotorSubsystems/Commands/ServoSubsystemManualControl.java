// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.Base.Motor.ServoSubsystemTalon;

public class ServoSubsystemManualControl extends Command {
  private ServoSubsystemTalon m_ServoSubsystemTalon;
  private DoubleSupplier m_precentOutput;
  boolean m_MotionMagic;
  /** Creates a new ServoSubsystemHoming. */
  
  public ServoSubsystemManualControl(ServoSubsystemTalon ServoSubsystemTalon , DoubleSupplier joystickPower) {
    m_ServoSubsystemTalon = ServoSubsystemTalon;
    m_precentOutput = joystickPower;
    addRequirements(ServoSubsystemTalon);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ServoSubsystemTalon.setPrecentOutput(m_precentOutput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ServoSubsystemTalon.disableMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
