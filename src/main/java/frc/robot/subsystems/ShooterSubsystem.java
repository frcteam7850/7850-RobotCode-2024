// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OperatorConstants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    //Creating Vars
       private CANSparkFlex ShooterMotor1;
       private CANSparkFlex ShooterMotor2;
       private RelativeEncoder ShooterMotor1Encoder;
       private RelativeEncoder ShooterMotor2Encoder;

    public ShooterSubsystem(){
        ShooterMotor1 = new CANSparkFlex(ShooterConstants.kMotor1ID, MotorType.kBrushless);
        ShooterMotor1Encoder = ShooterMotor1.getEncoder();   
        
        ShooterMotor2 = new CANSparkFlex(ShooterConstants.kMotor4ID, MotorType.kBrushless);
        ShooterMotor2Encoder = ShooterMotor2.getEncoder();

        ShooterMotor2.follow(ShooterMotor1, false);
    }

    //Functions
    //Start shooter code
    public void RunShooter(boolean speed){
      if (!speed){
        ShooterMotor1.set(ShooterConstants.kShooterSpeedLow);
      } else { 
        ShooterMotor1.set(ShooterConstants.kShooterSpeedHigh);
      }
    }
    //End shooter code

    public void StopMotor(){
        ShooterMotor1.stopMotor();
    }

    //Periodic Function
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Shooter Debug", ShooterMotor1Encoder.getPosition());
    }
}