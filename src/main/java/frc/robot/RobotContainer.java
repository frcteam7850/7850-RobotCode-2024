// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Imports
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import javax.swing.JToggleButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.OperatorConstants.ShooterConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.ArmConstants;

import frc.robot.commands.*;
import frc.robot.commands.ShooterIntakeCmd;
import frc.robot.commands.ShooterAmpCmd;
import frc.robot.commands.ShooterIntakeRevCmd;
import frc.robot.commands.ShooterSpeakerCmd;
import frc.robot.commands.ArmPIDZeroPositionCmd;
import frc.robot.commands.DebugRunMotorsCmd;
import frc.robot.commands.DebugRunMotorsNegCmd;
import frc.robot.commands.ArmPIDSourceAmpCmd;
import frc.robot.commands.ArmPIDSpeakerCmd;
import frc.robot.commands.TeleopSwerve;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

//Class
public class RobotContainer {

  //Stuff for arm and shooter using the logitech controller
  public final Joystick Stick = new Joystick(OperatorConstants.JoystickConstants.StickPort);

  private final JoystickButton  IntakeButton = new JoystickButton(Stick, ShooterConstants.IntakeButton);
  private final JoystickButton  IntakeRevButton = new JoystickButton(Stick, ShooterConstants.IntakeRevButton);
  private final JoystickButton  ShootAmpButton = new JoystickButton(Stick, ShooterConstants.shootAmp);
  private final JoystickButton  ShootSpeakerButton = new JoystickButton(Stick, ShooterConstants.shootSpeaker);
  private final JoystickButton  ShooterStopButton = new JoystickButton(Stick, 9);
  private final JoystickButton  IntakeStopButton = new JoystickButton(Stick, 10); // TODO: Check to see if correct, likely is 10


  private final CommandXboxController m_XboxController = new CommandXboxController(0);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  
  private final Trigger robotCentric =
  new Trigger(m_XboxController.leftBumper());

  // No idea why this is commented out 
    /*private final int translationAxis = Joystick.AxisType.kY.value; //left flight stick
    private final int strafeAxis = Joystick.AxisType.kX.value; //left flight stick
    private final int rotationAxis = Joystick.AxisType.kX.value; //right flight stick*/

  /* Subsystems */
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  public RobotContainer() {
  //Delcaring commands for Pathplanner to be able to use them (See pathplanner docs for for more info)
  NamedCommands.registerCommand("ArmPIDSourceAmpCmd", new ArmPIDSourceAmpCmd(m_ArmSubsystem));
  NamedCommands.registerCommand("ShooterSpeakerCmd(START)", new ShooterSpeakerCmd(m_ShooterSubsystem, false));
  NamedCommands.registerCommand("ShooterSpeakerCmd(STOP)", new ShooterSpeakerCmd(m_ShooterSubsystem, true));
  NamedCommands.registerCommand("ShooterIntakeCmd(START)", new ShooterSpeakerCmd(m_ShooterSubsystem, false));
  NamedCommands.registerCommand("ShooterIntakeCommand(STOP)", new ShooterIntakeCmd(m_IntakeSubsystem, true));
  NamedCommands.registerCommand("ArmPIDZeroPositionCmd", new ArmPIDZeroPositionCmd(m_ArmSubsystem));

  // Configure the trigger bindings (Swerve)
  configureBindings();

    m_SwerveSubsystem.setDefaultCommand(
      new TeleopSwerve(
          m_SwerveSubsystem,
          () -> -m_XboxController.getRawAxis(translationAxis),
          () -> -m_XboxController.getRawAxis(strafeAxis),
          () -> -m_XboxController.getRawAxis(rotationAxis),
          () -> robotCentric.getAsBoolean()));

    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    //Bindings for swerve stuff
     m_XboxController.button(Button.kY.value).onTrue(new InstantCommand(() -> m_SwerveSubsystem.zeroGyro()));
     m_XboxController.button(Button.kB.value).onTrue(new InstantCommand(() -> m_SwerveSubsystem.setWheelsToX()));

    // Bindings for our grabber/shooter 
     IntakeButton.onTrue(new ShooterIntakeCmd(m_IntakeSubsystem, true));
     ShootAmpButton.onTrue(new ShooterAmpCmd(m_ShooterSubsystem, true));
     IntakeRevButton.onTrue(new ShooterIntakeRevCmd(m_IntakeSubsystem, true));
     ShootSpeakerButton.onTrue(new ShooterSpeakerCmd(m_ShooterSubsystem, true));
     ShooterStopButton.onTrue(new ShooterAmpCmd(m_ShooterSubsystem, false));
     IntakeStopButton.onTrue(new ShooterIntakeCmd(m_IntakeSubsystem, false));

     IntakeButton.onFalse(new ShooterIntakeCmd(m_IntakeSubsystem, false));
    //  ShootAmpButton.onFalse(new ShooterAmpCmd(m_ShooterSubsystem, false));
     IntakeRevButton.onFalse(new ShooterIntakeRevCmd(m_IntakeSubsystem, false));
    //  ShootSpeakerButton.onFalse(new ShooterSpeakerCmd(m_ShooterSubsystem, false));
    
    // //Bindings for arm
    new JoystickButton(Stick, ArmConstants.ArmPIDButtonValue1).onTrue(new ArmPIDZeroPositionCmd(m_ArmSubsystem));
    new JoystickButton(Stick, ArmConstants.ArmPIDButtonValue2).onTrue(new ArmPIDSourceAmpCmd(m_ArmSubsystem));
    new JoystickButton(Stick, ArmConstants.ArmPIDButtonValue3).onTrue(new ArmPIDSpeakerCmd(m_ArmSubsystem));

    // //Debug controls to run the arm motors manually using the Logitech controller. This will be left in but used in the event of emergency. 

      // new JoystickButton(Stick, ArmConstants.ArmPIDButtonValue1).whileTrue(new DebugRunMotorsCmd(m_ArmSubsystem));
      // new JoystickButton(Stick, ArmConstants.ArmPIDButtonValue2).whileTrue(new DebugRunMotorsNegCmd(m_ArmSubsystem));
    }

    //Our auto commands. 
    public Command getAutonomousCommand() {
        // Load the path you want to follow using its name in the GUI

      //For following a single path:
        //Typically you want to define your path as a variable who's contents are Pathplanner.fromPathFile("Path Name")
        //After that you actually return AutoBuilder.followPath(variable)

      //For an Auto:
        //return new PathPlannerAuto("AutoName")
        
       return new PathPlannerAuto("Test_Auto");
    }    
  }

  