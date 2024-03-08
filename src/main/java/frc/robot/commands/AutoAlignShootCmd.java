// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.ShooterLookupTable;
import frc.robot.Constants.ShooterConstants;
import frc.robot.ShooterLookupTable.ActionSetpoint;

public class AutoAlignShootCmd extends Command {
  private final LimelightSubsystem limelightSubsystem;
  private final PivotSubsystem pivotSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterLookupTable shooterLookupTable;

  public AutoAlignShootCmd(LimelightSubsystem limelightSubsystem, PivotSubsystem pivotSubsystem, ShooterSubsystem shooterSubsystem) {
    this.limelightSubsystem = limelightSubsystem;
    this.pivotSubsystem = pivotSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    shooterLookupTable = new ShooterLookupTable();

    addRequirements(limelightSubsystem, pivotSubsystem, shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double distance = Math.round(limelightSubsystem.mean()*10)/10;


    ActionSetpoint pair = shooterLookupTable.get(distance);

    // double rpm = ShooterConstants.kShooterMotorSpeed;
    double rpm = -pair.getShooterRPM();
    double angle = pair.getPivotAngle() - 9;
    

    SmartDashboard.putNumber("Auto Align Des Angle", angle);
    SmartDashboard.putBoolean("April Tag Distance Zero???", distance != 0.0);

    shooterSubsystem.setShooterMotor(rpm);
    if(distance != 0.0) {
      pivotSubsystem.goToSetpoint(angle);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
