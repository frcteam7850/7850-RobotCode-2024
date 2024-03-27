// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//To eplain, we need to rev up the shooter before activating the inake, shooting that through the shooter. 
//This means we'd preferably like to have both running at once, but if they're both in the same subsystem, only
//one function could be run at a time. Hence, two subsystems for intake and shooter. 

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants.ArmConstants;
import frc.robot.Constants.OperatorConstants.ShooterConstants;

public class IntakeSubsystem extends SubsystemBase {
    //Creating Vars
      private CANSparkFlex IntakeMotor1;
      private CANSparkFlex IntakeMotor2;
      private RelativeEncoder IntakeMotor1Encoder;
      private RelativeEncoder IntakeMotor2Encoder;

    public IntakeSubsystem(){
      IntakeMotor1 = new CANSparkFlex(ShooterConstants.kMotor2ID, MotorType.kBrushless);
      IntakeMotor1Encoder = IntakeMotor1.getEncoder();        
        
      IntakeMotor2 = new CANSparkFlex(ShooterConstants.kMotor3ID, MotorType.kBrushless);
      IntakeMotor2Encoder = IntakeMotor2.getEncoder();    

      IntakeMotor2.follow(IntakeMotor1, false);
    }

    //Start shooter code
    public void RunShooterIntake(boolean reversed){
      if (!reversed){
        IntakeMotor1.set(ShooterConstants.kIntakeSpeed);
      } else { 
        IntakeMotor1.set(ShooterConstants.kIntakeRevSpeed);
      }
    }
    //End shooter code

    public void StopMotor(){
        IntakeMotor1.stopMotor();
    }

    //Periodic Function
    @Override
    public void periodic() {
    }
}