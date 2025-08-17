// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Sysid;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class PhoneixSysid {

    private SysIdRoutine routine;

    public PhoneixSysid(SysidConfiguration configuration, MotorSubsystemTalon subsystem) {
        this.routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        configuration.rampRate, // Default ramp rate is acceptable
                        configuration.stepVoltage, // Reduce dynamic voltage to 4 to prevent motor brownout
                        configuration.timeout, // Default timeout is acceptable
                        configuration.recordState), // Log state with Phoenix SignalLogger class
                new SysIdRoutine.Mechanism(
                        (volts) -> {
                            subsystem.runCharacterization(volts);
                        },
                        configuration.log,
                        subsystem));
        SignalLogger.setPath("sysid");
    }

    public Command runFullCharacterization(boolean wait) {
        if (wait) {
            return Commands.sequence(
                    new InstantCommand(() -> SignalLogger.start()),
                    this.routine.dynamic(Direction.kForward),
                    new WaitCommand(1),
                    this.routine.dynamic(Direction.kReverse),
                    new WaitCommand(1),
                    this.routine.quasistatic(Direction.kForward),
                    new WaitCommand(1),
                    this.routine.quasistatic(Direction.kReverse),
                    new InstantCommand(() -> SignalLogger.stop()));
        } else {
            return Commands.sequence(new InstantCommand(() -> SignalLogger.start()),
                    this.routine.dynamic(Direction.kForward),
                    this.routine.dynamic(Direction.kReverse),
                    this.routine.quasistatic(Direction.kForward),
                    this.routine.quasistatic(Direction.kReverse),
                    new InstantCommand(() -> SignalLogger.stop()));
        }
    }
}
