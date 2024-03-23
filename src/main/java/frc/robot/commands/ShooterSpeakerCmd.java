// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//Class
public class ShooterSpeakerCmd extends Command {
private final ShooterSubsystem m_ShooterSubsystem;
private boolean shooterButtonPressed;
public boolean ButtonStatusVar;

 public ShooterSpeakerCmd(ShooterSubsystem subsytem, boolean released) {
    m_ShooterSubsystem = subsytem;
    shooterButtonPressed = released;

    //Required Subsystem
    addRequirements(subsytem);
  }

  @Override
  public void initialize() {
  }

  public void ButtonStatus(boolean status){
    if (status){
        ButtonStatusVar = true;
    } else{
        ButtonStatusVar = false;
    }
    SmartDashboard.putBoolean("Button 4 Status", ButtonStatusVar);
  }

  @Override
  public void execute() {
    if(shooterButtonPressed == true){
     m_ShooterSubsystem.RunShooter(true);
     ButtonStatus(true);
    }
    else {
     m_ShooterSubsystem.StopMotor();
     ButtonStatus(false);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.StopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

