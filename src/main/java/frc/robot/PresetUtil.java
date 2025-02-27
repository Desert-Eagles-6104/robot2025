// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class PresetUtil {
    private double m_elevatorHeight;
    private double m_armAngle;

    public PresetUtil(double elevatorHeight,double armAngle){
        m_elevatorHeight = elevatorHeight;
        m_armAngle = armAngle;
    }
    
    public enum PresetState{
        L1,
        L2,
        L3,
        L4,
        Home,
        Human,
        Intake;
    }

    public double getArmAngle() {
        return m_armAngle;
    }

    public double getElevatorHeight() {
        return m_elevatorHeight;
    }

    public static PresetUtil getPresetState(PresetState state){
        switch (state) {

            case L1:
                return new PresetUtil(0.2,-54.208984375);//first stage   
            
            case L2:
                return new PresetUtil(0.6,-54.208984375);//second stage   
            
            case L3:
                return new PresetUtil(0.8,-54.208984375);//thired stage   
            
            case L4:
                return new PresetUtil(0.9,-54.208984375);//4 stage   
            
            case Home:
                return new PresetUtil(0.1,89.384765625);//home stage   
            
            case Human:
                return new PresetUtil(0.0,37);//coralStation   0.07 elevator pos
            
            case Intake:
                return new PresetUtil(0.1,-54.208984375);   
            
            default:
                return new PresetUtil(0.0,89.384765625);   
        
        }
    }

}

