// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.commands;

import frc.robot.Constants.OperatorConstants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

//Class
public class AutoArmPIDZeroPositionCmd extends Command {
  private boolean finish = false;

  private final ArmSubsystem m_ArmSubsystem;

  public AutoArmPIDZeroPositionCmd(ArmSubsystem subsytem) {
    m_ArmSubsystem = subsytem;
    //Required Subsystem
    addRequirements(subsytem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
      finish = false;
      m_ArmSubsystem.SetPosition(ArmConstants.kZeroSetpoint);
      if(((ArmConstants.kZeroSetpoint + ArmConstants.kPIDShutdownRange) > (ArmSubsystem.GetEncoderPos())) && ((ArmConstants.kZeroSetpoint - ArmConstants.kPIDShutdownRange) < (ArmSubsystem.GetEncoderPos()))){
        finish = true;
      }
    //Will equal 0 degrees
  }

  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.StopMotor();
    System.out.println("finish1");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("y");
    return finish;
  }
}

