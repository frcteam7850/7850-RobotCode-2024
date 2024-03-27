// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.SwerveModuleConstants;

import frc.robot.RobotContainer;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants extends RobotContainer{
  public static final class SwerveConstants{
    public static final double inputDeadband = .1;
    public static final boolean invertnavx = false;

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(23);//to find
    public static final double wheelBase = Units.inchesToMeters(23);//to find
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (8.14 / 1.0); // 6.75:1 L2 Mk4 Modules
    //L1 is 8.14:1, L2 is 6.75:1, L3 is 6.12:1, L4 is 5.14:1
    public static final double angleGearRatio = (21.43 / 1.0); // 12.8:1 MK4 SDS Modules

    public static final SwerveDriveKinematics swerveKinematics =
    new SwerveDriveKinematics(
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0), //module 0 //translation 2d locates the swerve module in cords
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), //module 1
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), //module 2
        new Translation2d(wheelBase / 2.0,  trackWidth / 2.0)); //module 3
    //https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html
    //SwerveDrive Kinematics converts between a ChassisSpeeds object and several SwerveModuleState objects, 
    //which contains velocities and angles for each swerve module of a swerve drive robot.
        
    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;
       
    //Swerve Current Limiting for neos
    public static final int angleContinuousCurrentLimit = 20; //limits current draw of turning motor
    public static final int driveContinuousCurrentLimit = 40; //limits current draw of drive motor
  


    /* Drive Motor PID Values */
    public static final double driveKP = 0.1; //to tune
    public static final double driveKI = 0.0; //to tune
    public static final double driveKD = 0.0; //to tune
   public static final double driveKFF = 0.0; //to tune

    /* Drive Motor Characterization Values */
    //values to calculate the drive feedforward (KFF)
    public static final double driveKS = 0.667; //to calculate
    public static final double driveKV = 2.44; //to calculate
    public static final double driveKA = 0.27; //to calculate

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
    (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 9; // meters per second
    public static final double maxAngularVelocity = 11.5; //what are these units?

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;
    

    /* Module Specific Constants */
    /* Back Right Module - CANcoder 8 - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 7; 
      public static final int angleMotorID = 9; 
      public static final int canCoderID = 8;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(347.7);
    /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
    
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
          //creates a constant with all info from swerve module
    }

    /* Back Left Module - CANcoder 5 - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 5;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(196.8);
      /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
        
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
          //creates a constant with all info from swerve module
    }

    /* Front Right Module - CANcoder 11 - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(212.2);
      /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
  
      public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
        //creates a constant with all info from swerve module
    }

    /* Front Left Module - CANcoder 2 - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 2 ;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(174.2);
        /* Angle Motor PID Values */
      public static final double angleKP = 0.01; //to tune
      public static final double angleKI = 0.0; //to tune
      public static final double angleKD = 0.0; //to tune
      public static final double angleKFF = 0.0; //to tune
    
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset, angleKP, angleKI, angleKD, angleKFF);
          //creates a constant with all info from swerve module
    }
  

    public static final boolean angleMotorInvert = false;
    public static final boolean driveMotorInvert = false;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1.75;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5; //Changed 3/9/24
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static class OperatorConstants {
    public static class JoystickConstants {
      public static final int StickPort = 1; //If your controller is unresponsive, make sure you plugged in the XBOX controller first (Stickport 0), and then the logitech (Stickport 1)
      public static final double kDeadband = 0.12;
      public static final int kArmYAxis = 1;
    }

    public static class ArmConstants {
      //PID Tuning Values
      public static final double Kp = 12;  //tune these bro :sob:
      public static final double Ki = 0;
      public static final double Kd = .1;
      public static final double kFF = 0;  //We may not need a FeedForward. 

      public static final double kZeroSetpoint = 0;
      public static final double kSourceSetpoint = 0.19;
      public static final double kSpeakerSetpoint = 0.5;

      // Inverted?
      public static final boolean ArmMotor1IsInverted = true;
      public static final boolean ArmMotor2IsInverted = true;

      //CanIDS 
      public static final int kArmMotor1ID = 45; 
      public static final int kArmMotor2ID = 44; 
      public static final int kClimberMotor1ID = 51;
      public static final int kClimberMotor2ID = 52;

      //Misc
      public static final double NegMaxPIDRange = -0.5; 
      public static final double PosMaxPIDRange = 0.5;

      public static final int kAltEncoderCountsPerRev = 4096;

      public static final double kArmSpeed = 0.2;

      //Controller button vals
      public static final int ArmPIDButtonValue1 = 1; //Button assignment on the shooter/arm controller
      public static final int ArmPIDButtonValue2 = 2; //Button assignment on the shooter/arm controller 
      public static final int ArmPIDButtonValue3 = 3; //Button assignment on the shooter/arm controller     

    }

    public static class ShooterConstants {

      //Ids
      public static final int kMotor1ID = 50; 
      public static final int kMotor2ID = 51; 
      public static final int kMotor3ID = 52;
      public static final int kMotor4ID = 53;

      //Speeds
      public static final double kShooterSpeedLow = -0.2; 
      public static final double kShooterRevSpeed = 0.3;
      public static final double kIntakeSpeed = -0.2; 

      public static final double kShooterSpeedHigh = -1; 
      public static final double kIntakeRevSpeed = 0.35; 

      //Button Assignments
      public static final int IntakeButton = 8; //Button assignment on the shooter/arm controller
      public static final int IntakeRevButton = 6; //Button assignment on the shooter/arm controller 

      public static final int shootAmp = 5; //Button assignment on the shooter/arm controller
      public static final int shootSpeaker = 7; //Button assignment on the shooter/arm controller 

    
    }
    public static class ClimberConstants {
      //CanIDs
      public static final int kClimberMotor1ID = 51;
      public static final int kClimberMotor2ID = 52;

      //Speed
      public static final double kClimberSpeed = 0.3;

      //Button Assignment 
      public static final double kClimberSafetyButton = 9;
      public static final double kClimberButton = 9;

      //Distance (in motor rotations)
      public static final int kMotorRotations = 10; //Change

      //PID
      public static final double Kp = 12;
      public static final double Ki = 0;
      public static final double Kd = .1;
    }
  }
}

