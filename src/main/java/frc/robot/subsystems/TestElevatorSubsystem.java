package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib25.Sensors.BeamBreak;
import frc.DELib25.Subsystems.ServoSubsystem.ServoSubsystemConfiguration;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class TestElevatorSubsystem extends ElevatorSubsystem {
    public TestElevatorSubsystem(ServoSubsystemConfiguration configuration) {
        super(configuration);
    }
}
