package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

/**
 * This command is used to home a motor subsystem by setting it to a specific position
 * and applying a resistive output until the motor reaches a certain current threshold
 */
public class MotorSubsystemHoming extends Command {
  private MotorSubsystemTalon servoSubsystemTalon;
  private final double resistPrecent;
  private final double currentThreshold;
  private final double velocityThreshold;
  
  /** Creates a new ServoSubsystemHoming. 
   * @param velocityThreshold 
   */
  public MotorSubsystemHoming(MotorSubsystemTalon motorSubsystemTalon, double resistPrecent, double currentThreshold, double velocityThreshold) {
    this.servoSubsystemTalon = motorSubsystemTalon;
    this.resistPrecent = resistPrecent;
    this.currentThreshold = currentThreshold;
    this.velocityThreshold = velocityThreshold;
    addRequirements(motorSubsystemTalon);
  }

  @Override
  public void initialize() {
    this.servoSubsystemTalon.ControlSoftLimit(false);
    this.servoSubsystemTalon.setPosition(this.servoSubsystemTalon.configuration.homePosition);
  }

  @Override
  public void execute() {
    if (this.servoSubsystemTalon.isAtSetpoint()) {
      this.servoSubsystemTalon.setPrecentOutput(this.resistPrecent);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.servoSubsystemTalon.ControlSoftLimit(true);
    this.servoSubsystemTalon.disableMotors();
  }

  @Override
  public boolean isFinished() {
    return this.servoSubsystemTalon.getMotorCurrent() > this.currentThreshold && this.servoSubsystemTalon.getVelocity() < this.velocityThreshold;
  }
}