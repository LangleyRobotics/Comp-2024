// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootAutoCmd extends Command {
  private final ShooterSubsystem shooterSubsystem;

  public ShootAutoCmd(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;

    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    shooterSubsystem.setShooterMotor(-ShooterConstants.kShooterMotorSpeed);
  }
   
  @Override
  public void execute() {
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

