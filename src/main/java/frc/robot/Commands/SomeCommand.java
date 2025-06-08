package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;
import frc.DELib25.Subsystems.ServoSubsystem.Commands.ServoSubsystemSetPosition;
import frc.robot.subsystems.TestServoSubsystem;

public class SomeCommand extends ServoSubsystemSetPosition{
    TestServoSubsystem m_servo = new TestServoSubsystem();
    public SomeCommand(ServoSubsystemTalon ServoSubsystemTalon, double Position) {
        super(ServoSubsystemTalon, Position);
    }
    
}
