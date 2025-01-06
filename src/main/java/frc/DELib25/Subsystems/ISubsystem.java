// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Voltage;

public interface ISubsystem {
  /**
   * call this function in the constructor to reset the system in the start (Zero Sensors)
   */
  public void resetSubsystemToInitialState();

  /**
   * disable all the subsystem motors when called
   */
  public abstract void disableMotors();

  /**
   * base function for sysid characterization
   * @param volts
   */
  public abstract void runCharacterization(Voltage volts);;
}