// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.MathMethods;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.CameraSubsystem;

/*
 * NOTE:
 * 0 degrees = arm back parallel to ground
 * 180 degrees = intake position
 * Get degrees from encoder output
 * Must calibrate encoder to that range = set offset and distance per rotation (144)
 */

public class PivotControllerCmd extends Command{

  private final PivotSubsystem pivotSubsystem;
  private final CameraSubsystem cameraSubsystem;
  private final Supplier<Double> pivotPositiveDirFunction;
  private final Supplier<Double> pivotNegativeDirFunction;

  public PivotControllerCmd(PivotSubsystem pivotSubsystem, Supplier<Double> pivotPositiveDirFunction, Supplier<Double> pivotNegativeDirFunction) {
    this.pivotSubsystem = pivotSubsystem;
    this.cameraSubsystem = null;
    this.pivotPositiveDirFunction = pivotPositiveDirFunction;
    this.pivotNegativeDirFunction = pivotNegativeDirFunction;

    addRequirements(pivotSubsystem);
  }

  public PivotControllerCmd(PivotSubsystem pivotSubsystem, CameraSubsystem cameraSubsystem) {
    this.pivotSubsystem = pivotSubsystem;
    this.cameraSubsystem = cameraSubsystem;
    this.pivotPositiveDirFunction = () -> 0.0;
    this.pivotNegativeDirFunction = () -> 0.0;

    addRequirements(pivotSubsystem, cameraSubsystem);
  }

  @Override
  public void initialize() {

  }


  @Override
  public void execute() {

    //If setting pivot to a setpoint
    if(cameraSubsystem != null) {
      pivotSubsystem.goToSetpoint(MathMethods.calculateSetpoint(cameraSubsystem));
      //pivotSubsystem.pivotWithFeedforwardPID(double desPosition, double desVelocity, double desAccel);
    } else {
      
      //Establish button inputs
      // double positiveSpeed = pivotPositiveDirFunction.get();
      // double negativeSpeed = pivotNegativeDirFunction.get();

      // double velocity = positiveSpeed - negativeSpeed;

      //pivotSubsystem.setPivotMotor(velocity);



      //**Pivot toward intake position**
      if(pivotPositiveDirFunction.get() > 0) {
        //pivotSubsystem.pivotWithFeedforwardPID(Math.PI, velocity, PivotConstants.kPivotMotorAccel, 90);
        if(pivotSubsystem.getPivotAbsEncoder() > 90) {
          pivotSubsystem.setPivotMotorNoBounds(0.3);
        } else if(pivotSubsystem.getPivotAbsEncoder() < 45) {
            pivotSubsystem.setPivotMotorNoBounds(0.05);
        } else {
          pivotSubsystem.setPivotMotorNoBounds(0.15);
        }
      } 
      
      
      //**Pivot toward amp position**
      else if(pivotNegativeDirFunction.get() > 0) {
        //pivotSubsystem.pivotWithFeedforwardPID(Math.PI / -4, velocity, PivotConstants.kPivotMotorAccel, 90);
        if(pivotSubsystem.getPivotAbsEncoder() < 90) {
          pivotSubsystem.setPivotMotorNoBounds(-0.3);
        } else if(pivotSubsystem.getPivotAbsEncoder() > 115) {
            pivotSubsystem.setPivotMotorNoBounds(0);
        } else {
          pivotSubsystem.setPivotMotorNoBounds(-0.15);
        }
      }
      
      
      //**Keep arm in place when no controller input**
      else {
        pivotSubsystem.setPivotMotorNoBounds(MathMethods.signDouble(Math.cos(pivotSubsystem.getPivotAbsEncoder()))*0.02-PivotConstants.pivotCompensation * Math.cos(Math.toRadians(pivotSubsystem.getPivotAbsEncoder())));
      }
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
