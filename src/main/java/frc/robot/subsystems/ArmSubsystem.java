// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;

import com.revrobotics.CANEncoder;  //Not unused. Don't believe the lies. 
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.ArmConstants;

//Class
public class ArmSubsystem extends SubsystemBase {
    //Creating Vars
      //Arm
      private CANSparkMax ArmMotor1;
       private RelativeEncoder ArmMotor1Encoder;

       private CANSparkMax ArmMotor2;
       private RelativeEncoder ArmMotor2Encoder;

      //PID Control
       private final SparkPIDController PIDMotorController;

      //Rev Hex Encoder
       static private CANEncoder AbsoluteEncoder;
    
    public ArmSubsystem(){
       // //Instantiating Vars

        ArmMotor1 = new CANSparkMax(ArmConstants.kArmMotor1ID, MotorType.kBrushless);
        ArmMotor1Encoder = ArmMotor1.getEncoder();

         ArmMotor2 = new CANSparkMax(ArmConstants.kArmMotor2ID, MotorType.kBrushless);
         ArmMotor2Encoder = ArmMotor2.getEncoder();

         //Ensuring they're in break mode (This may not actually work, and we should test to make sure. Otherwise, do it through rev.)
         ArmMotor1.setIdleMode(CANSparkBase.IdleMode.kBrake);
         ArmMotor2.setIdleMode(CANSparkBase.IdleMode.kBrake);

         // Check to see if inverted
         ArmMotor1.setInverted(ArmConstants.ArmMotor1IsInverted);
         ArmMotor2.setInverted(ArmConstants.ArmMotor2IsInverted);

         PIDMotorController = ArmMotor1.getPIDController();

         AbsoluteEncoder = ArmMotor1.getAlternateEncoder(ArmConstants.kAltEncoderCountsPerRev);

           //It's strange that it only takes one argument. On the docs it should take the encoder type as the 
           //first arg, however, i believe this is because types of alt encoders other than quadrature aren't
           //supported currently. So, if this breaks in the future, try seeing if REV added the alt encoder type arg.

         //coefficients
         PIDMotorController.setP(ArmConstants.Kp);
         PIDMotorController.setI(ArmConstants.Ki);
         PIDMotorController.setD(ArmConstants.Kd);
           //Didn't feel it was neccessary to add a Feed Forward (see command below) or closed loop cont. for the abs encoder.
         // PIDMotorController.setFF(ArmConstants.kFF);
         PIDMotorController.setOutputRange(ArmConstants.NegMaxPIDRange, ArmConstants.PosMaxPIDRange);

        //setting motors to follow
        //pass true for second argument to make the motors spin in the same direction
        ArmMotor2.follow(ArmMotor1, false);

        //Setting the PID listening device to the Absolute Encoder. If this is removed, the PID command will intake Motor Rotations instead of Abs Encoder Position
        PIDMotorController.setFeedbackDevice(AbsoluteEncoder);
    }

    //Functions
    //Main PID function. 
    public void SetPosition(double setpoint){
           PIDMotorController.setReference(setpoint + 0.04, CANSparkBase.ControlType.kPosition);
    }

    static public double GetEncoderPos(){
      return AbsoluteEncoder.getPosition();
    }

    //Start debug code
    public void DebugRunArmPositive(){
      ArmMotor1.set(ArmConstants.kArmSpeed);
    }

    public void DebugRunArmNegative(){
      ArmMotor1.set(-ArmConstants.kArmSpeed);
    }
    //End debug code

    //Start value getting funcs
    public double GetMotor1Velocity(){
    return ArmMotor1Encoder.getVelocity();
    }

    public double GetMotor2Velocity(){
    return ArmMotor2Encoder.getVelocity();
    }

    public double GetMotor1Position(){
    return ArmMotor1Encoder.getPosition();
    }

    public double GetMotor2Position(){
    return ArmMotor2Encoder.getPosition();
    }

    public void StopMotor(){
    ArmMotor1.stopMotor();
    }

    //Periodic Function
    @Override
    public void periodic() {
      SmartDashboard.putNumber("Output", ArmMotor1.getAppliedOutput());
      SmartDashboard.putNumber("Rotations", GetMotor1Position());
      SmartDashboard.putNumber("#Rotations", GetMotor1Position());
      SmartDashboard.putNumber("kP", OperatorConstants.ArmConstants.Kp);
      SmartDashboard.putNumber("kI", OperatorConstants.ArmConstants.Ki);
      SmartDashboard.putNumber("kD", OperatorConstants.ArmConstants.Kd);
      SmartDashboard.putNumber("Abs Encoder Position Raw", AbsoluteEncoder.getPosition());
      SmartDashboard.putNumber("Abs Encoder Position", AbsoluteEncoder.getPosition()*180);
    }
}