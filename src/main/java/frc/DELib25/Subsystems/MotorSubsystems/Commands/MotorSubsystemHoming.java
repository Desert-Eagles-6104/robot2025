package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

/**
 * This command is used to home a motor subsystem by setting it to a specific position
 * and applying a resistive output until the motor reaches a certain current threshold
 */
public class MotorSubsystemHoming extends Command {
  private MotorSubsystemTalon subsystemTalon;
  private final double resistPrecent;
  private final double currentThreshold;
  private final double velocityThreshold;
  
  /** Creates a new ServoSubsystemHoming. 
   * @param velocityThreshold 
   */
  public MotorSubsystemHoming(MotorSubsystemTalon subsystemTalon, double resistPrecent, double currentThreshold, double velocityThreshold) {
    this.subsystemTalon = subsystemTalon;
    this.resistPrecent = resistPrecent;
    this.currentThreshold = currentThreshold;
    this.velocityThreshold = velocityThreshold;
    addRequirements(subsystemTalon);
  }

  @Override
  public void initialize() {
    this.subsystemTalon.ControlSoftLimit(false);
    this.subsystemTalon.setPosition(this.subsystemTalon.configuration.homePosition);
  }

  @Override
  public void execute() {
    if (this.subsystemTalon.isAtSetpoint()) {
      this.subsystemTalon.setPrecentOutput(this.resistPrecent);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.subsystemTalon.ControlSoftLimit(true);
    this.subsystemTalon.disableMotors();
  }

  @Override
  public boolean isFinished() {
    return this.subsystemTalon.getMotorCurrent() > this.currentThreshold && this.subsystemTalon.getVelocity() < this.velocityThreshold;
  }
}