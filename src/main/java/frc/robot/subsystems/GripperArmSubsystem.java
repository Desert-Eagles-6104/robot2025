// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class GripperArmSubsystem extends ServoSubsystemTalon {
  CANcoder m_absoluteEncoder;
  /** Creates a new CoralArmSubsystem. */
  public GripperArmSubsystem(ServoSubsystemConfiguration configuration) {
    super(configuration);
    m_absoluteEncoder = new CANcoder(9);
  }

  @Override
  public void periodic() {
    super.periodic();
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
}
