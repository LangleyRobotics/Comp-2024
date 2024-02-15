// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

//Set pivot to specific setpoints (intake, shoot up close, amp scoring)

public class SetPivotCmd extends Command {
  
  private final PivotSubsystem pivotSubsystem;
  private final int position;

  public SetPivotCmd(PivotSubsystem pivotSubsystem, int position) {
    this.pivotSubsystem = pivotSubsystem;
    this.position = position;

    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    if(position == 0) {
      pivotSubsystem.goToSetpoint(PivotConstants.kMinPivotPosition);
    } else if (position == 1) {
      pivotSubsystem.goToSetpoint(PivotConstants.shootUpClosePosition);
    } else if (position == 2) {
      pivotSubsystem.goToSetpoint(PivotConstants.kMaxPivotPosition);
    }
  }

  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.stopPivotMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
