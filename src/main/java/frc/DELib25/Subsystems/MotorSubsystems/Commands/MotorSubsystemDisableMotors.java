package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemDisableMotors extends InstantCommand {
  private MotorSubsystemTalon subsystemTalon;

  public MotorSubsystemDisableMotors(MotorSubsystemTalon subsystemTalon) {
    this.subsystemTalon = subsystemTalon;
    addRequirements(subsystemTalon);
  }

  @Override
  public void initialize() {
    this.subsystemTalon.disableMotors();
  }
}