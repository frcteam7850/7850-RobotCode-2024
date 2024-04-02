// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

//Class
public class ClimberUpCmd extends Command {
private final ClimberSubsystem m_ClimberSubsystem;

  public ClimberUpCmd(ClimberSubsystem subsytem) {
    m_ClimberSubsystem = subsytem;
    //Required Subsystem
    addRequirements(subsytem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_ClimberSubsystem.StartClimber(0);
  }

  @Override
  public void end(boolean interrupted) {
    m_ClimberSubsystem.StopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

