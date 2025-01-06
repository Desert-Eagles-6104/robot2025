// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.DELib25.Sysid;

// import java.util.function.BooleanSupplier;
// import java.util.function.Consumer;
// import com.ctre.phoenix6.SignalLogger;
// import edu.wpi.first.units.Measure;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import frc.DELib25.Subsystems.ISubsystem;
// import frc.DELib25.Subsystems.ServoSubsystem.Base.IServoSubsystemBase;
// import frc.DELib25.Subsystems.VelocitySubsystem.Base.IVelocitySubsystemBase;

// /** Add your docs here. */
// public class SparkSysid {
//     private SysIdRoutine m_routine;
//      boolean startLogging = false;

//     public SparkSysid(SysidConfiguration configuration, SubsystemBase subsystem, Consumer<Voltage> runCharacterization){
//         m_routine =  new SysIdRoutine(
//             new SysIdRoutine.Config(
//                 configuration.m_rampRate,  // Default ramp rate is acceptable
//                 configuration.m_stepVoltage, // Reduce dynamic voltage to 4 to prevent motor brownout
//                 configuration.m_timeout,          // Default timeout is acceptable
//                 configuration.m_recordState), // Log state with Phoenix SignalLogger class
//             new SysIdRoutine.Mechanism(
//                 runCharacterization,
//                 configuration.m_log,
//                 subsystem));
//         SignalLogger.setPath("sysid");
//     }

//     public SparkSysid(SysidConfiguration configuration, IServoSubsystemBase subsystem){
//         m_routine =  new SysIdRoutine(
//             new SysIdRoutine.Config(
//                 configuration.m_rampRate,  // Default ramp rate is acceptable
//                 configuration.m_stepVoltage, // Reduce dynamic voltage to 4 to prevent motor brownout
//                 configuration.m_timeout,          // Default timeout is acceptable
//                 configuration.m_recordState), // Log state with Phoenix SignalLogger class
//             new SysIdRoutine.Mechanism(
//                 (volts)-> {
//                     subsystem.runCharacterization(volts);
//                 },
//                 configuration.m_log,
//                 new SubsystemBase() {
//                 }));
//         SignalLogger.setPath("sysid");
//     }

//         public SparkSysid(SysidConfiguration configuration, IVelocitySubsystemBase subsystem){
//         m_routine =  new SysIdRoutine(
//             new SysIdRoutine.Config(
//                 configuration.m_rampRate,  // Default ramp rate is acceptable
//                 configuration.m_stepVoltage, // Reduce dynamic voltage to 4 to prevent motor brownout
//                 configuration.m_timeout,          // Default timeout is acceptable
//                 configuration.m_recordState), // Log state with Phoenix SignalLogger class
//             new SysIdRoutine.Mechanism(
//                 (volts)-> {
//                     subsystem.runCharacterization(volts);
//                 },
//                 configuration.m_log,
//                 new SubsystemBase() {
//                 }));
//         SignalLogger.setPath("sysid");
//     }

//         public SparkSysid(SysidConfiguration configuration, ISubsystem subsystem){
//         m_routine =  new SysIdRoutine(
//             new SysIdRoutine.Config(
//                 configuration.m_rampRate,  // Default ramp rate is acceptable
//                 configuration.m_stepVoltage, // Reduce dynamic voltage to 4 to prevent motor brownout
//                 configuration.m_timeout,          // Default timeout is acceptable
//                 configuration.m_recordState), // Log state with Phoenix SignalLogger class
//             new SysIdRoutine.Mechanism(
//                 (volts)-> {
//                     subsystem.runCharacterization(volts);
//                 },
//                 configuration.m_log,
//                 new SubsystemBase() {
//                 }));
//         SignalLogger.setPath("sysid");
//     }



//     public Command runFullCharacterization(boolean wait){
//         if(wait){
//             return Commands.sequence(new InstantCommand(() -> SignalLogger.start()),
//             new InstantCommand(() -> startLogging = true), 
//             m_routine.dynamic(Direction.kForward),
//             new WaitCommand(1),
//             m_routine.dynamic(Direction.kReverse),
//             new WaitCommand(1),
//             m_routine.quasistatic(Direction.kForward),
//             new WaitCommand(1),
//             m_routine.quasistatic(Direction.kReverse),
//             new InstantCommand(() -> startLogging = false),
//             new InstantCommand(() -> SignalLogger.stop()));
//             }   
//         else{
//             return Commands.sequence(new InstantCommand(() -> SignalLogger.start()), 
//             m_routine.dynamic(Direction.kForward),
//             m_routine.dynamic(Direction.kReverse),
//             m_routine.quasistatic(Direction.kForward),
//             m_routine.quasistatic(Direction.kReverse),
//             new InstantCommand(() -> SignalLogger.stop()));
//             }
//         }

//         /**
//          * must be called in periodic
//          * @param motorVoltage
//          * @param motorPosition
//          * @param motorVelocity
//          */
//         public void log(double motorVoltage, double motorPosition, double motorVelocity){
//             if(startLogging){
//             SignalLogger.writeDouble("motorVoltage" , motorVoltage);
//             SignalLogger.writeDouble("motorPosition", motorPosition);
//             SignalLogger.writeDouble("motorVelocity", motorVelocity);
//             }
//         }

//         public BooleanSupplier getStartLogging(){
//             return () -> startLogging;
//         }
// }
