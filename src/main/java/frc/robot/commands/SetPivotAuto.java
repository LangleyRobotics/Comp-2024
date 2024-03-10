// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.ShooterLookupTable;
import frc.robot.commands.MoveToSetPointCmd;
import frc.robot.subsystems.GoodPivot;

//Set pivot to specific setpoints (intake, shoot up close, amp scoring)

public class SetPivotAuto extends Command {
  
  private final PivotSubsystem pivotSubsystem;
  //private final GoodPivot pivot;
  private final int position;
  private final double targetPosition;

  public SetPivotAuto(PivotSubsystem pivotSubsystem, int position) {
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
      if(Math.abs(this.position-targetPosition)<=5){
        end(true);
      }
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
