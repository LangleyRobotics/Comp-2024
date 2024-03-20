// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAlignAmpCmd extends Command {
  private final LimelightSubsystem limelightSubsystem;
  private final DriveSubsystem driveSubsystem;
  private SlewRateLimiter turningLimiter = new SlewRateLimiter(Constants.kMaxAngularAccelerationRadiansPerSecondSquared);

  public AutoAlignAmpCmd(DriveSubsystem driveSubsystem, LimelightSubsystem limelightSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.limelightSubsystem = limelightSubsystem;

    addRequirements(driveSubsystem, limelightSubsystem);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    double theta = limelightSubsystem.getThetaToTarget();
    double xDist = limelightSubsystem.getDistanceToTarget() * Math.sin(theta);
    double yDist = limelightSubsystem.getDistanceToTarget() * Math.cos(theta);

    if(theta > Math.toRadians(10)) {
      double turningSpeed = Math.sin(theta);
      turningSpeed = turningLimiter.calculate(turningSpeed)
                * Constants.kMaxAngularSpeedRadiansPerSecond;

      ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xDist * 0.5, yDist * 0.5, turningSpeed);

      SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

      driveSubsystem.setModuleStates(moduleStates);

    }
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
