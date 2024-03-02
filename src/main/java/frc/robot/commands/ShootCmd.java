// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;
  private final boolean finished;

  public ShootCmd(ShooterSubsystem shooterSubsystem, boolean finished) {
    this.shooterSubsystem = shooterSubsystem;
    this.finished = finished;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(finished) {
      shooterSubsystem.setShooterMotor(0.0);
    } else {
      shooterSubsystem.setShooterMotor(ShooterConstants.kShooterMotorSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooterMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
