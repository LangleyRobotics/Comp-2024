// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.function.Supplier;

//Will shoot the ring during autonomous

public class ShootDuringAutoCmd extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ShootDuringAutoCmd(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterMotor(ShooterConstants.kShooterMotorSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooterMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}