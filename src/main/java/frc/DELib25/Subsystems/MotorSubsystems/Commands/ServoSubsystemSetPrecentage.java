package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class ServoSubsystemSetPrecentage extends InstantCommand {
  private MotorSubsystemTalon m_ServoSubsystemTalon;
  private double m_precentOutput;
  
  public ServoSubsystemSetPrecentage(MotorSubsystemTalon ServoSubsystemTalon, double precentOutput) {
    m_ServoSubsystemTalon = ServoSubsystemTalon;
    m_precentOutput = precentOutput;
    addRequirements(ServoSubsystemTalon);
  }

  @Override
  public void initialize() {
    m_ServoSubsystemTalon.setPrecentOutput(m_precentOutput);
  }
}
