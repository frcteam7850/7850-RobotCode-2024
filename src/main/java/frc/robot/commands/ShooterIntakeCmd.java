// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//Class
public class ShooterIntakeCmd extends Command {
private final IntakeSubsystem m_IntakeSubsystem;
private boolean intakeButtonPressed;
public boolean ButtonStatusVar;

 public ShooterIntakeCmd(IntakeSubsystem subsytem, boolean released) {
    m_IntakeSubsystem = subsytem;
    intakeButtonPressed = released;

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
    SmartDashboard.putBoolean("Button 3 Status", ButtonStatusVar);
  }

  @Override
  public void execute() {
    if(intakeButtonPressed == true){
     m_IntakeSubsystem.RunShooterIntake(false);
     ButtonStatus(true);
    }
    else {
     m_IntakeSubsystem.StopMotor();
     ButtonStatus(false);
    }
  }
  
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.StopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

