// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot.commands;

import frc.robot.Constants.OperatorConstants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.lang.Math;

//Class
public class ArmPIDSourceAmpCmd extends Command {

  // private double timeAtSetpoint;
  // private double targetTime = 50;
  private boolean finish = false;

  private final ArmSubsystem m_ArmSubsystem;

  public ArmPIDSourceAmpCmd(ArmSubsystem subsytem) {
    m_ArmSubsystem = subsytem;
    //Required Subsystem
    addRequirements(subsytem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    if (Math.abs(m_ArmSubsystem.GetEncoderPos() - ArmConstants.kZeroSetpoint) < 0.01) finish = true;

      m_ArmSubsystem.SetPosition(ArmConstants.kSpeakerSetpoint);
    //Will equal 0 degrees
  }

  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.StopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("ZERO END");
    return finish;
  }
}

