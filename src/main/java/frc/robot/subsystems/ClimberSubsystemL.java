// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants.ClimberConstants;

public class ClimberSubsystemL extends SubsystemBase {
    //Creating Vars
       private CANSparkMax ClimberMotorR;
       private RelativeEncoder ClimberMotorREncoder;

       public boolean Safety = false;

    public ClimberSubsystemL(){
        ClimberMotorR = new CANSparkMax(ClimberConstants.kClimberMotor2ID, MotorType.kBrushless);
        ClimberMotorREncoder = ClimberMotorR.getEncoder();
    }

    //Functions
    //Start climber code
    public void Safety(boolean status){
        Safety = status;
    System.out.println(Safety);
    }
   
   //Start shooter code
    public void RunPositive(){
        if(Safety){
         ClimberMotorR.set((ClimberConstants.kClimberSpeed) * -1);
        }
      }
  
      public void RunNegative(){
       if(Safety){
         ClimberMotorR.set(ClimberConstants.kClimberSpeed);
        }
      }
    //End climber code

    public void StopMotor(){
        ClimberMotorR.stopMotor();
    }

    //Periodic Function
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climber Safety ()", Safety);
    }
}