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
public class ArmPIDSpeakerCmd extends Command {

  // private double timeAtSetpoint;
  // private double targetTime = 50;
  private boolean finish = false;

  private final ArmSubsystem m_ArmSubsystem;

  public ArmPIDSpeakerCmd(ArmSubsystem subsytem) {
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
      m_ArmSubsystem.SetPosition(ArmConstants.kSourceSetpoint);
    //Will equal 0 degrees
  }

  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.StopMotor();
    System.out.println("SPEAKER END");
 }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
