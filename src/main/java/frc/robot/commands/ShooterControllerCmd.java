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
  private final Supplier<Boolean> shooterHalfPositiveDirFunction;
  private final Supplier<Boolean> shooterHalfNegativeDirFunction;
  
  public ShooterControllerCmd(ShooterSubsystem shooterSubsystem, Supplier<Boolean> shooterNegativeDirFunction, Supplier<Boolean> shooterPositiveDirFunction,Supplier<Boolean> shooterHalfNegativeDirFunction,Supplier<Boolean> shooterHalfPositiveDirFunction) {
    this.shooterSubsystem = shooterSubsystem;
    this.shooterNegativeDirFunction = shooterNegativeDirFunction;
    this.shooterPositiveDirFunction = shooterPositiveDirFunction;
    this.shooterHalfNegativeDirFunction = shooterHalfNegativeDirFunction;
    this.shooterHalfPositiveDirFunction = shooterHalfPositiveDirFunction;

    addRequirements(shooterSubsystem);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    
    //are the buttons pressed
    boolean positiveDir = shooterPositiveDirFunction.get();
    boolean negativeDir = shooterNegativeDirFunction.get();
    boolean halfNegativeDir = shooterHalfNegativeDirFunction.get();
    boolean halfPositiveDir = shooterHalfPositiveDirFunction.get();

    double velocity = 0;

    if(positiveDir) {
      velocity = ShooterConstants.kShooterMotorSpeed;
    } else if(negativeDir) {
      velocity = -1 * ShooterConstants.kShooterMotorSpeed;
    } else if(halfNegativeDir) {
      velocity = ShooterConstants.kShooterMotorSpeed/1.25;
    } else if(halfPositiveDir) {
      velocity = -1 * ShooterConstants.kShooterMotorSpeed/1.25;
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
