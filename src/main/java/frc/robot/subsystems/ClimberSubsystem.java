// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants.ArmConstants;
import frc.robot.Constants.OperatorConstants.ClimberConstants;
import frc.robot.Constants.OperatorConstants.ShooterConstants;
import frc.robot.commands.ClimberSafetyCmd;;

public class ClimberSubsystem extends SubsystemBase {
    //Creating Vars
       private CANSparkFlex ClimberMotor1;
       private CANSparkFlex ClimberMotor2;
       private RelativeEncoder ClimberMotor1Encoder;
       private RelativeEncoder ClimberMotor2Encoder;

       private final SparkPIDController PIDMotorController;

       public boolean Safety = false;

    public ClimberSubsystem(){

        ClimberMotor1 = new CANSparkFlex(ClimberConstants.kClimberMotor1ID, MotorType.kBrushless);
        ClimberMotor1Encoder = ClimberMotor1.getEncoder();   
        
        ClimberMotor2 = new CANSparkFlex(ClimberConstants.kClimberMotor2ID, MotorType.kBrushless);
        ClimberMotor2Encoder = ClimberMotor2.getEncoder();

        PIDMotorController = ClimberMotor1.getPIDController();

        PIDMotorController.setP(ClimberConstants.Kp);
        PIDMotorController.setI(ClimberConstants.Ki);
        PIDMotorController.setD(ClimberConstants.Kd);    
        
        ClimberMotor2.follow(ClimberMotor1, true); 
    }

    //Functions
    //Start climber code
    public void Safety(boolean status){
        Safety = status;
    System.out.println(Safety);
    }
   
    public void StartClimber(double setpoint){
      if (Safety){
        PIDMotorController.setReference(setpoint, CANSparkBase.ControlType.kPosition);
      } else {
        System.out.println("Could not move climber");
      }
    }

    //End climber code

    public void StopMotor(){
        ClimberMotor1.stopMotor();
    }

    //Periodic Function
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climber Safety ()", Safety);
    }
}