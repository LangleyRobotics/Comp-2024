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
  private final double targetPosition;

  public SetPivotCmd(PivotSubsystem pivotSubsystem, int position) {
    this.pivotSubsystem = pivotSubsystem;
    this.position = position;

    if(position == 0) {
      //**Intake position**
      this.targetPosition = PivotConstants.kMaxPivotPosition;
    } else if (position == 1) {
      //**Shoot position for up close**
      this.targetPosition = PivotConstants.shootUpClosePosition;
    } else if (position == 2) {
      //**Amp scoring position**
      this.targetPosition = PivotConstants.kAmpPosition;
    } else if(position == 3) {
      //**Shoot position for ring 1**
      this.targetPosition = PivotConstants.shootRing1;
    } else if(position == 4) {
      //**Shoot position for side of subwoofer**
      this.targetPosition = PivotConstants.shootSidePosition;
    } else if(position == 5) {
      this.targetPosition = PivotConstants.shoot301;
    } else {
      this.targetPosition = PivotConstants.kMaxPivotPosition;
    }

    addRequirements(pivotSubsystem);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
      pivotSubsystem.goToSetpoint(targetPosition);
  }

  @Override
  public void end(boolean interrupted) {
    pivotSubsystem.stopPivotMotor();
  }

  @Override
  public boolean isFinished() {
    return pivotSubsystem.isAtSetpoint();
  }
}
