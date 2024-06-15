// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This is a debug function for the arm motors, delete before final build

//Imports
package frc.robot.commands;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

//Class
public class DebugRunMotorsCmd extends Command {
private final ArmSubsystem m_ArmSubsystem;

  public DebugRunMotorsCmd(ArmSubsystem subsytem) {
    m_ArmSubsystem = subsytem;
    //Required Subsystem
    addRequirements(subsytem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_ArmSubsystem.DebugRunArmPositive();
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

