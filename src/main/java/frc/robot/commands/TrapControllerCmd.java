// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import frc.robot.Constants.TrapConstants;
import frc.robot.subsystems.TrapSubsytem;

public class TrapControllerCmd extends Command {
  private final TrapSubsytem trap;
  private final Supplier<Boolean> trapTurn;
  private final boolean forward;

  public TrapControllerCmd(TrapSubsytem trap,Supplier<Boolean> trapTurn,boolean forward) {
    this.trap = trap;
    this.trapTurn = trapTurn;
    this.forward = forward;

    addRequirements(trap);
  }

  @Override
  public void initialize() {}


  @Override
  public void execute() {
    boolean run = trapTurn.get();
    if(run){
      if(forward) {
        trap.setTrap(TrapConstants.kTrapSpeed);
      }
      else {
        trap.setTrap(-TrapConstants.kTrapSpeed);
      }
    }
  }


  @Override
  public void end(boolean interrupted) {
    trap.stopTrap();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
