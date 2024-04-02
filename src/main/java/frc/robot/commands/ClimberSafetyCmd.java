// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

//Class
public class ClimberSafetyCmd extends Command {
private boolean status = true;
private boolean end = false;
private final ClimberSubsystem m_ClimberSubsystem;

  public ClimberSafetyCmd(ClimberSubsystem subsytem) {
    m_ClimberSubsystem = subsytem;
    //Required Subsystem
    addRequirements(subsytem);
  }

  @Override
  public void initialize() {
  if (status){
      status = false;
  } else{
      status = true;
  }
  m_ClimberSubsystem.Safety(status);
  end = true;
  }

  @Override
  public void execute() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}

