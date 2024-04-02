// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.commands;

import frc.robot.Constants.OperatorConstants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

//Class
public class ArmPIDZeroPositionCmd extends Command {

  private final ArmSubsystem m_ArmSubsystem;

  public ArmPIDZeroPositionCmd(ArmSubsystem subsytem) {
    m_ArmSubsystem = subsytem;
    //Required Subsystem
    addRequirements(subsytem);
  }

  @Override
  public void initialize() {
  }
//haha
  @Override
  public void execute() {
      m_ArmSubsystem.SetPosition(ArmConstants.kZeroSetpoint - 0.2); //Check the need for this offset before starting the robot. 
    //Will equal 0 degrees
  }

  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.StopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

