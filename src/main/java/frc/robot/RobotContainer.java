// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.swerve.*;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.Constants.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


public class RobotContainer {
  protected SwerveDrive<REVSwerveModule> swervedrive;
  protected SwerveDriveCommand swervedrivecommand;
  protected XboxController xbox = new XboxController(0);
  protected final AHRS m_gyro = new AHRS(SPI.Port.kMXP);


  public RobotContainer() {
    REVSwerveModule m_frontLeft = new REVSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      DriveConstants.frontleft_translation);

    REVSwerveModule m_frontRight = new REVSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      DriveConstants.frontright_translation);

    REVSwerveModule m_rearLeft = new REVSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      DriveConstants.rearleft_translation);

    REVSwerveModule m_rearRight = new REVSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      DriveConstants.rearright_translation);


    swervedrive = new SwerveDrive<>(m_gyro, m_frontLeft, m_frontRight, m_rearLeft, m_rearRight);
    swervedrivecommand = new SwerveDriveCommand(swervedrive, xbox, DriveConstants.DriveAugParams);
    // SwerveDrive<SwerveModule> test = (SwerveDrive<SwerveModule>)swervedrive; <-- "to remember", this test actsing doesn't work
    swervedrive.register();

    swervedrive.smartDashboardInit("Swerve Drive");
    SmartDashboard.putData("Swerve Drive Command", swervedrivecommand);

    swervedrive.setDefaultCommand(swervedrivecommand);
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
