package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class HomingElevator extends Command {
  private MotorSubsystemTalon elevator;
  private double m_currentThreshold = 0.98; //TODO: need to be in const
  private double m_velocityThreshold = 0.1; //TODO: need to be in const
  private StableBoolean condition;
  public HomingElevator(MotorSubsystemTalon elevator) {
    this.elevator = elevator;
    this.condition = new StableBoolean(0.2);
    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    this.elevator.ControlSoftLimit(false);
    this.elevator.setPrecentOutput(-0.05);
  }


  @Override
  public void end(boolean interrupted) {
    this.elevator.resetPosition(0);
    this.elevator.setPrecentOutput(0.0);
  }

  @Override
  public boolean isFinished() {
    return this.condition.update(this.elevator.getMotorCurrent() > m_currentThreshold && this.elevator.getVelocity() < m_velocityThreshold);
  }
}