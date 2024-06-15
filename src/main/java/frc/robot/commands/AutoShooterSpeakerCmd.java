// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import javax.swing.plaf.basic.BasicTreeUI.TreeToggleAction;

import edu.wpi.first.wpilibj2.command.Command;

//Class
public class AutoShooterSpeakerCmd extends Command {
private final ShooterSubsystem m_shooterSubsystem;
private boolean Status;
private boolean finish = false;

 public AutoShooterSpeakerCmd(ShooterSubsystem subsytem, boolean status) {
    m_shooterSubsystem = subsytem;
    Status = status;
    
    //Required Subsystem
    addRequirements(subsytem);
  }

  @Override
  public void initialize() {
    if(Status){
     m_shooterSubsystem.RunShooter(true);
    }
    else {
     m_shooterSubsystem.StopMotor();
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

