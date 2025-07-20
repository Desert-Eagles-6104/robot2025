package frc.DELib25.Subsystems.MotorSubsystems.VelocitySubsystem.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib25.Subsystems.MotorSubsystems.VelocitySubsystem.Base.Motor.VelocitySubsystemTalon;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
public class VelocitySubsystemDisableMotors extends InstantCommand {
  private VelocitySubsystemTalon subsystemTalon;

  public VelocitySubsystemDisableMotors(VelocitySubsystemTalon velocitySubsystemTalon) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(velocitySubsystemTalon);
    this.subsystemTalon = velocitySubsystemTalon;
  }

  @Override
  public void initialize() {
    this.subsystemTalon.disableMotors();
  }
}