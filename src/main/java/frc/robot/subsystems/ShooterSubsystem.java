// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;


public class ShooterSubsystem extends SubsystemBase {
  
  private final CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.kShooterMotorPort, MotorType.kBrushless);

  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Motor Velocity", shooterMotor.get());
  }

  public void setShooterMotor(double percentage) {
    shooterMotor.set(percentage);
  
    // try {
    //   shooterMotor.set(percentage);

    // } catch(Exception e) {
    //     System.out.println("Error: Shooter Motor exception:" + e.toString());
    // }
  }

  public void stopShooterMotor() {
    shooterMotor.set(0);
  }
  
}
