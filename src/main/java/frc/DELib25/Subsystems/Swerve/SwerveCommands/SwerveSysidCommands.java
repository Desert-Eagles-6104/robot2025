package frc.DELib25.Subsystems.Swerve.SwerveCommands;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;

public class SwerveSysidCommands {
    private SysIdRoutine m_routine;

    public SwerveSysidCommands(SwerveSubsystem swerve){
        m_routine =  new SysIdRoutine(
                new SysIdRoutine.Config(
                    Volts.of(4).div(Seconds.of(4)),  // Default ramp rate is acceptable
                    Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                    Seconds.of(6), // Default timeout is acceptable
                                        // Log state with Phoenix SignalLogger class
                    (state)->SignalLogger.writeString("state", state.toString())),
                new SysIdRoutine.Mechanism(
                    (volts)-> {
                        swerve.runSwerveCharacterization(volts);
                    },
                    null,
                    swerve));
                    
        SignalLogger.setPath("sysid");
    }


    //important!!! you need to run atleast 3 different tests - there 2 types, each type has for amd back
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Commands.sequence(
            new InstantCommand(() -> SignalLogger.start()), 
            m_routine.quasistatic(direction),
            new InstantCommand(() -> SignalLogger.stop())
            );
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return Commands.sequence(
            new InstantCommand(() -> SignalLogger.start()), 
            m_routine.dynamic(direction),
            new InstantCommand(() -> SignalLogger.stop())
            );
    }

    /**
     * runs a full test of sysid 2 forward and 2 backward
     * @return a sequenceal command
     */
    public Command fullSysidRun(){
        return Commands.sequence(new InstantCommand(() -> SignalLogger.start()), 
            m_routine.dynamic(Direction.kForward),
            m_routine.dynamic(Direction.kReverse),
            m_routine.quasistatic(Direction.kForward),
            m_routine.quasistatic(Direction.kReverse),
            new InstantCommand(() -> SignalLogger.stop()));
    }

    
}
