// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.integrationCommands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GripperArmSubsystem;
import frc.robot.subsystems.GripperSubsystem;
import frc.robot.presetState;
import frc.robot.presetState.PresetState;
import frc.robot.subsystems.ElevatorSubsystem;

public class SmartPreset extends Command {
  private ElevatorSubsystem m_Elevator;
  private GripperArmSubsystem m_gripperArm;
  private GripperSubsystem m_gripper;
  private PresetState m_state;

  /** Creates a new command to simply control set preset of the robot with all its subsystems. */
  public SmartPreset(ElevatorSubsystem elevatorSubsystem , GripperArmSubsystem gripperArmSubsystem, GripperSubsystem gripper ,PresetState state) {
    m_Elevator = elevatorSubsystem;
    m_gripperArm = gripperArmSubsystem;
    m_gripper = gripper;
    m_state = state;
    addRequirements(m_Elevator);
  }

  // Called when the command is initially scheduled.
  public void initialize() {
    
    m_gripperArm.setMotionMagicPosition(presetState.getPresetState(m_state).getArmAngle());
    m_Elevator.setMotionMagicPosition(presetState.getPresetState(m_state).getElevatorHeight());

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Elevator.isAtSetpoint() && m_gripperArm.isAtSetpoint();
  }
  
  private void set(PresetState state){
    if(m_gripper.HasGamePiece()){
      m_gripper.setMotorPercent(0.0);
      m_gripperArm.setMotionMagicPosition(presetState.getPresetState(state).getArmAngle());
      m_Elevator.setMotionMagicPosition(presetState.getPresetState(state).getElevatorHeight());
    }
    else{
     m_gripper.setMotorPercent(0.0);
     m_gripperArm.setMotionMagicPosition(presetState.getPresetState(PresetState.Human).getArmAngle());
     m_Elevator.setMotionMagicPosition(presetState.getPresetState(PresetState.Human).getElevatorHeight());
    } 
  }
}