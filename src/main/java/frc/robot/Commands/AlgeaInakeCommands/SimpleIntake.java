package frc.robot.Commands.AlgeaInakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.LatchedBolean;
import frc.robot.subsystems.AlgaeIntakeSubsystem;

public class SimpleIntake extends Command {
  /** Creates a new SimpleIntake. */
  private AlgaeIntakeSubsystem m_intake;
  private boolean first = true;
  private LatchedBolean m_firstStage;
  private Timer m_timer;
  private int i = 0;
  private double intakePower = 0.3;
  
  
  public SimpleIntake(AlgaeIntakeSubsystem intake) {
    m_intake = intake;
    m_firstStage = new LatchedBolean();
    m_timer = new Timer();
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_firstStage.reset();
    m_timer.reset();
    m_timer.start();
    first = true;
    i = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_firstStage.update(m_intake.getBeamBreak()); 
    if(first){
      m_intake.setMotorPrecent(intakePower);
      first = !m_firstStage.get();
      m_timer.reset();
    }
    else{
      m_intake.setMotorPrecent(0);
    }
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
