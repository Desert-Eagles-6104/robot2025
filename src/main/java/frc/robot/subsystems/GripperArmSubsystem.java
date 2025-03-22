// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class GripperArmSubsystem extends ServoSubsystemTalon {
  CANcoder m_absoluteEncoder;
  double m_angleOffset;
  /** Creates a new CoralArmSubsystem. */
  public GripperArmSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);
    m_absoluteEncoder = new CANcoder(9);
    m_angleOffset = m_configuration.angleOffset;

    m_angleOffset = m_configuration.angleOffset;
    // setAngleOffset(m_configuration.angleOffset);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("CanCoderValue", m_absoluteEncoder.getPosition().getValueAsDouble());
  }

  @Override
  public void setPosition(double position){
    if(Math.abs(super.getClosedLoopError()) < 2){
      super.setPosition(position);
    }
    else{
      super.setMotionMagicPosition(position);
    }
  }

  public double getAbsAngleWithOffset(){
        return m_absoluteEncoder.getAbsolutePosition().getValueAsDouble() - m_angleOffset;
    }

    public void resetToAbsolute(){
      setPosition(getAbsAngleWithOffset());
  }

   public void setAngleOffset(double angleOffset){
        m_angleOffset = angleOffset;
        resetToAbsolute();
    }

    @Override
  public void resetSubsystemToInitialState() {
    resetPosition(m_configuration.homePosition);
  }

  
  }

