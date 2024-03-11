package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAutoCmd extends Command{
    private final IntakeSubsystem intakeSubsystem;
    private final double speed;
    //direction int is negative 1 or 1.
    private final int direction;

    public IntakeAutoCmd(IntakeSubsystem intakeSubsystem, double speed, int direction) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        this.direction = direction;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        int dir = direction;
        double velocity = speed*dir;

        intakeSubsystem.setIntakeMotor(velocity);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }



}
