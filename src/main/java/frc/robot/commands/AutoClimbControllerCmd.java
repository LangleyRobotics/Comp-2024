// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;


public class AutoClimbControllerCmd extends Command{

  private final ClimbSubsystem climbSubsystem;
  private final Supplier<Boolean> climbDirFunction;
  private final boolean up;
  private final String which;


  public AutoClimbControllerCmd(ClimbSubsystem climbSubsystem, Supplier<Boolean> climbDirFunction, boolean up, String which) {
    this.climbSubsystem = climbSubsystem;
    this.climbDirFunction = climbDirFunction;
    this.up = up;
    this.which = which;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    
    //is the button pressed
    boolean dir = climbDirFunction.get();

    double velocity = 0;

    if(dir) {
      if(up) {
        velocity = ClimbConstants.kClimbMotorSpeed;
        if(which.equals("left")) {
          climbSubsystem.setLeftClimbMotor(velocity);
        } else if(which.equals("right")){
          climbSubsystem.setRightClimbMotor(velocity);
        } else if(which.equals("both")){
          climbSubsystem.setRightClimbMotor(velocity);
          climbSubsystem.setLeftClimbMotor(velocity);
        }
      } else {
          velocity = -1 * ClimbConstants.kClimbMotorSpeed;
          if(which.equals("left")) {
            climbSubsystem.setLeftClimbMotor(velocity);
          } else if(which.equals("right")) {
            climbSubsystem.setRightClimbMotor(velocity);
          } else if (which.equals("both")) {
            climbSubsystem.setRightClimbMotor(velocity);
            climbSubsystem.setLeftClimbMotor(velocity);
          }
      }
    }if(climbSubsystem.getleftLimitSwitch()){
      end(false);
      }
  }

  @Override
  public void end(boolean interrupted) {
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
