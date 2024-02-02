// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;


public class ClimbControllerCmd extends Command{

  private final ClimbSubsystem climbSubsystem;
  private final Supplier<Boolean> climbPositiveDirFunction;
  private final Supplier<Boolean> climbNegativeDirFunction;

  public ClimbControllerCmd(ClimbSubsystem climbSubsystem, Supplier<Boolean> climbPositiveDirFunction, Supplier<Boolean> climbNegativeDirFunction) {
    this.climbSubsystem = climbSubsystem;
    this.climbPositiveDirFunction = climbPositiveDirFunction;
    this.climbNegativeDirFunction = climbNegativeDirFunction;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    
    //is the button pressed
    boolean positiveDir = climbPositiveDirFunction.get();
    boolean negativeDir = climbNegativeDirFunction.get();

    double velocity = 0;

    if(positiveDir) {
      velocity = ClimbConstants.kClimbMotorSpeed;
    } else if(negativeDir) {
      velocity = -1 * ClimbConstants.kClimbMotorSpeed;
    }

    climbSubsystem.setClimbMotor(velocity);
  }


  @Override
  public void end(boolean interrupted) {
    climbSubsystem.stopClimbMotor();
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
