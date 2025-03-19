// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class presetState extends SubsystemBase {
    private double m_elevatorHeight;
    private double m_armAngle;

    public presetState(double elevatorHeight,double armAngle){
        m_elevatorHeight = elevatorHeight;
        m_armAngle = armAngle;
    }
    
    public enum PresetState{
        AlgeL2,
        L2,
        L3,
        L4,
        Home,
        Human,
        AlgeL3,
        ZERO,
        Intake;
    }

    public double getArmAngle() {
        return m_armAngle;
    }

    public double getElevatorHeight() {
        return m_elevatorHeight;
    }

    public static presetState getPresetState(PresetState state){
        switch (state) {

            case ZERO:
                return new presetState(0.00001,-86);

            case AlgeL2:
                return new presetState(0.17,-15.208984375);//first stage  
            
            case AlgeL3:
                return new presetState(0.37,-15.208984375);//first stage   
            
            case L2:
                return new presetState(0.18,-44.208984375);//second stage   
            
            case L3:
                return new presetState(0.38,-44.208984375);//thired stage   
            
            case L4:
                return new presetState(0.70,-42.208984375);//4 stage   
            
            case Home:
                return new presetState(0.04,-89.384765625);//home stage   
            
            case Human:
                return new presetState(0.09,35.8);//coralStation   
            
            case Intake:
                return new presetState(0.1,-54.208984375);   
            
            default:
                return new presetState(0.07,35.5);   
        
        }
    }

}

