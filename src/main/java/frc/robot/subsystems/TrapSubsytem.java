// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TrapConstants;

public class TrapSubsytem extends SubsystemBase {
   private final CANSparkMax trapMotor = new CANSparkMax(TrapConstants.kTrapMotorPort, MotorType.kBrushless);
  /** Creates a new TrapSubsytem. */
  public TrapSubsytem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setTrap(double velocity){
    trapMotor.set(velocity);
  }
  public void stopTrap(){
    trapMotor.set(0);
  }
}
