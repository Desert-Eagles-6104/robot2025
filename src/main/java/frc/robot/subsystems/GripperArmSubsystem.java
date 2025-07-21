package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemConfiguration;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class GripperArmSubsystem extends MotorSubsystemTalon {
  CANcoder m_absoluteEncoder;
  double m_angleOffset;

  /** Creates a new CoralArmSubsystem. */
  public GripperArmSubsystem(MotorSubsystemConfiguration configuration) {
    super(configuration);
    m_absoluteEncoder = new CANcoder(9);
    m_angleOffset = this.configuration.angleOffset;

    m_angleOffset = this.configuration.angleOffset;
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("CanCoderValue", m_absoluteEncoder.getPosition().getValueAsDouble());
  }

  @Override
  public void setPosition(double position) {
    if (Math.abs(super.getClosedLoopError()) < 2) {
      super.setPosition(position);
    } else {
      super.setMotionMagicPosition(position);
    }
  }

  public double getAbsAngleWithOffset() {
    return m_absoluteEncoder.getAbsolutePosition().getValueAsDouble() - m_angleOffset;
  }

  public void resetToAbsolute() {
    setPosition(getAbsAngleWithOffset());
  }

  public void setAngleOffset(double angleOffset) {
    m_angleOffset = angleOffset;
    resetToAbsolute();
  }

  @Override
  public void resetSubsystemToInitialState() {
    resetPosition(this.configuration.homePosition);
  }

}
