// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterControllerCmd extends Command {
  
  private final ShooterSubsystem shooterSubsystem;
  private final Supplier<Boolean> shooterPositiveDirFunction;
  private final Supplier<Boolean> shooterNegativeDirFunction;

  
  public ShooterControllerCmd(ShooterSubsystem shooterSubsystem, Supplier<Boolean> shooterPositiveDirFunction, Supplier<Boolean> shooterNegativeDirFunction) {
    this.shooterSubsystem = shooterSubsystem;
    this.shooterPositiveDirFunction = shooterPositiveDirFunction;
    this.shooterNegativeDirFunction = shooterNegativeDirFunction;

    addRequirements(shooterSubsystem);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    
    //are the buttons pressed
    boolean positiveDir = shooterPositiveDirFunction.get();
    boolean negativeDir = shooterNegativeDirFunction.get();

    double velocity = 0;

    if(positiveDir) {
      velocity = ShooterConstants.kShooterMotorSpeed;
    } else if(negativeDir) {
      velocity = -1 * ShooterConstants.kShooterMotorSpeed;
    }

    shooterSubsystem.setShooterMotor(velocity);
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
