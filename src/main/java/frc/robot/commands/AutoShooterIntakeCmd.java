// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import javax.swing.plaf.basic.BasicTreeUI.TreeToggleAction;

import edu.wpi.first.wpilibj2.command.Command;

//Class
public class AutoShooterIntakeCmd extends Command {
private final IntakeSubsystem m_IntakeSubsystem;
private boolean Status;
private boolean finish = false;

 public AutoShooterIntakeCmd(IntakeSubsystem subsytem, boolean status) {
    m_IntakeSubsystem = subsytem;
    Status = status;
    
    //Required Subsystem
    addRequirements(subsytem);
  }

  @Override
  public void initialize() {
    if(Status){
     m_IntakeSubsystem.RunShooterIntake(false);
    }
    else {
     m_IntakeSubsystem.StopMotor();
    }
    finish = true;
  }

  @Override
  public void execute() {
  }
  
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}

