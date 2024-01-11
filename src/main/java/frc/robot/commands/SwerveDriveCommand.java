// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.swerve.*;
import frc.robot.swerve.SwerveDrive.AugmentationParams;
import frc.robot.swerve.SwerveUtils.ChassisStates;

public class SwerveDriveCommand extends CommandBase {

  
  public static class PoseIntegrator {
    public Pose2d pose = new Pose2d();
    // returns pose2d
    public Pose2d iterate(ChassisStates v, double dt) {
        return this.iterate(
            v.x_velocity,
            v.y_velocity,
            v.angular_velocity,
            dt
        );
    }
    public Pose2d iterate(double vx, double vy, double vr, double dt) {
        final Twist2d dx = new Twist2d(
            vx * dt,
            vy * dt,
            vr * dt
        );
        return (this.pose = this.pose.exp(dx));
    }
  }


  protected SwerveDrive<SwerveModule> swervedrive;
  protected ChassisStates prev_target = new ChassisStates();
  protected XboxController xbox;
  protected AugmentationParams aug_params;
  protected PoseIntegrator pose_integ = new PoseIntegrator();
  
  /** Creates a new SwerveDriveCommand. */
  public <Module_T extends SwerveModule> SwerveDriveCommand(SwerveDrive<Module_T> sdrive, XboxController xboxc, AugmentationParams aug_params) {
    this.swervedrive = (SwerveDrive<SwerveModule>)sdrive;
    this.xbox = xboxc;
    this.aug_params = aug_params;
    

    this.addRequirements(swervedrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swervedrive.applyParams(aug_params);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xbox.getRightBumper()){
      swervedrive.applyLocked();
      prev_target.zero();
    }
    else
    {
      double vx = -MathUtil.applyDeadband(xbox.getRightY(), Constants.OIConstants.kDriveDeadband) 
                                            * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
      double vy = -MathUtil.applyDeadband(xbox.getRightX(), Constants.OIConstants.kDriveDeadband) 
                                            * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
      double vr = -MathUtil.applyDeadband(xbox.getLeftX(), Constants.OIConstants.kDriveDeadband) 
                                            * Constants.DriveConstants.kMaxAngularSpeed;

      ChassisStates target = new ChassisStates(vx, vy, vr);
      ChassisStates.rateLimitVelocities(
        target, prev_target, 0.02, 
        Constants.DriveConstants.MaxLinearAcceleration, Constants.DriveConstants.MaxRotationalAcceleration);
      ChassisStates.descretizeCurvature(target, 0.02);

      this.swervedrive.applyTarget(target); 
      this.prev_target = target;
    }

    this.pose_integ.iterate(this.prev_target, 0.02);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swervedrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }



  @Override
  public void initSendable(SendableBuilder b) {
    super.initSendable(b);
    b.addDoubleProperty("X Velocity", ()->this.prev_target.x_velocity, null);
    b.addDoubleProperty("Y Velocity", ()->this.prev_target.y_velocity, null);
    b.addDoubleProperty("Angular Velocity", ()->this.prev_target.angular_velocity, null);
    b.addDoubleArrayProperty("Target Pose", ()-> SwerveUtils.toComponents2d(pose_integ.pose), null);

  }
}
