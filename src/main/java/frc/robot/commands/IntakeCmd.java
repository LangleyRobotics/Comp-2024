package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCmd extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final Supplier<Double> speedSupplier;
    //direction int is negative 1 or 1.
    private final int direction;

    public IntakeCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> speedSupplier, int direction) {
        this.intakeSubsystem = intakeSubsystem;
        this.speedSupplier = speedSupplier;
        this.direction = direction;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double speed = speedSupplier.get();
        int dir = direction;
        double velocity = speed*dir;

        intakeSubsystem.setIntakeMotor(velocity);

    
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntakeMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }



}
