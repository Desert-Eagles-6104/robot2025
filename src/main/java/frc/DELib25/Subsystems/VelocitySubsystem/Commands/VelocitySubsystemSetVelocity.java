// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.VelocitySubsystem.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.VelocitySubsystem.Base.Motor.VelocitySubsystemTalon;

public class VelocitySubsystemSetVelocity extends Command {
  private VelocitySubsystemTalon m_VelocitySubsystemTalon;
  private double m_Veloicty;
  private boolean m_motioMagic = false;
  private DoubleSupplier m_value = null;
  
  /**
   * @param velocitySubsystemTalon
   * @param Velocity
   * uses velocity
   */
  public VelocitySubsystemSetVelocity(VelocitySubsystemTalon velocitySubsystemTalon , double Velocity) {
    m_VelocitySubsystemTalon = velocitySubsystemTalon;
    m_Veloicty = Velocity;
    addRequirements(velocitySubsystemTalon);
  }

  /** Creates a new VelocitySubsystemSetSpeed. 
   * choose to use motion magic
  */
  public VelocitySubsystemSetVelocity(VelocitySubsystemTalon velocitySubsystemTalon , double Velocity , boolean motionMagic) {
    m_VelocitySubsystemTalon = velocitySubsystemTalon;
    m_Veloicty = Velocity;
    addRequirements(velocitySubsystemTalon);
    m_motioMagic = motionMagic;
  }

    /** Creates a new VelocitySubsystemSetSpeed. 
     * uses interpulation
    */
    public VelocitySubsystemSetVelocity(VelocitySubsystemTalon velocitySubsystemTalon, boolean motionMagic, DoubleSupplier value) {
      m_VelocitySubsystemTalon = velocitySubsystemTalon;
      m_value = value;
      addRequirements(velocitySubsystemTalon);
      m_motioMagic = motionMagic;
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_value == null){
      if (m_motioMagic) {
        m_VelocitySubsystemTalon.setMotionMagicVelocity(m_Veloicty);
      }
      else{ 
        m_VelocitySubsystemTalon.setVelocity(m_Veloicty);
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_value != null){
      m_VelocitySubsystemTalon.setUsingInterpulation(m_value.getAsDouble(), m_motioMagic);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_value == null; //id its not using interpulation then adrees this command as an instant command
  }
}
