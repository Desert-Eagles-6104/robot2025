package frc.robot.subsystems;

import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;
import frc.DELib25.Subsystems.Swerve.SwerveConstants;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;

public class TestServoSubsystem extends ServoSubsystemTalon {
    public TestServoSubsystem() {
        super(new ServoSubsystemConfiguration());
    }
    @Override
    public void periodic() {
        super.periodic();
    }
}
