// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import java.util.function.Consumer;
import frc.robot.subsystems.GoodPivot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MoveToSetPointCmd extends Command {
  /** Creates a new MoveToSetPointCmd. */
  private PIDController pivotPidController;
  private double kDt;
  private Consumer<Double> targetMotorSpeed;
  private Supplier<Double> curPosition, curVelocity;

  private TrapezoidProfile.Constraints constraints;
  private TrapezoidProfile.State current, goal;

  private double tolerance;

  private double setpoint;

  public MoveToSetPointCmd(Consumer<Double> targetMotorSpeed, Supplier<Double>curPosition,
            Supplier<Double> curVelocity, PIDController pivotPidController, double kDt, double tolerance,
            TrapezoidProfile.Constraints constraints, double setpoint) {
        this.targetMotorSpeed = targetMotorSpeed;
        this.curPosition = curPosition;
        this.curVelocity = curVelocity;
        this.pivotPidController = pivotPidController;
        this.kDt = kDt;

        this.constraints = constraints;
        this.setpoint = setpoint;

        this.tolerance = tolerance;

    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    goal = new TrapezoidProfile.State(setpoint, 0);
    current = new TrapezoidProfile.State(curPosition.get(), curVelocity.get());
    updateCurrent();
}
private TrapezoidProfile getProfile() {
  return new TrapezoidProfile(constraints);
}
private void updateCurrent() {
  var profile = getProfile();
  current = profile.calculate(kDt, current, goal);
}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = pivotPidController.calculate(curPosition.get(), current.position);
        targetMotorSpeed.accept(speed);
        updateCurrent();
        SmartDashboard.putNumber("Encoder Position", curPosition.get());
        SmartDashboard.putNumber("Current Position", current.position);
        SmartDashboard.putNumber("Current Velocity", speed);
        SmartDashboard.putNumber("Target Position", goal.position);
        SmartDashboard.putBoolean("We done?", isDone()&& pivotPidController.atSetpoint());
        SmartDashboard.putNumber("Dist from target", Math.abs(setpoint - curPosition.get()));
  }

  private boolean isDone() {
    return Math.abs(setpoint - curPosition.get()) <= tolerance;
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    targetMotorSpeed.accept(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (isDone()) && pivotPidController.atSetpoint();
  }
}
