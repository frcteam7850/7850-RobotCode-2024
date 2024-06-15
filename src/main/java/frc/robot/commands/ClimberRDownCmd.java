// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//This is a debug function for the arm motors, delete before final build

//Imports
package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystemR;
import edu.wpi.first.wpilibj2.command.Command;

//Class
public class ClimberRDownCmd extends Command {
private final ClimberSubsystemR m_ClimberSubsystemR;

  public ClimberRDownCmd(ClimberSubsystemR subsytem) {
    m_ClimberSubsystemR = subsytem;
    //Required Subsystem
    addRequirements(subsytem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    System.out.println("rdo");
    m_ClimberSubsystemR.RunNegative();
  }

  @Override
  public void end(boolean interrupted) {
    m_ClimberSubsystemR.StopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

