// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.Constants.ClimbConstants;

public class ClimbAutoCmd extends Command {
    private final ClimbSubsystem climbSubsystem;
    private final boolean up;
    private final String which;

  /** Creates a new ClimbAutoCmd. */
  public ClimbAutoCmd(ClimbSubsystem climbSubsystem, boolean up, String which) {
    this.climbSubsystem = climbSubsystem;
    this.up = up;
    this.which = which;

    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//is the button pressed

    double velocity = 0;
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