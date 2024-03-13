package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class RumbleCmd extends Command{

    private final XboxController rumXboxController;
    //rumbleside: 1 = Left, 2 = Right, 3 = Both
    private final int rumbleSide;
    private final double power;

    public RumbleCmd(XboxController rumXboxController, int rumbleSide, double power) {
        this.rumXboxController = rumXboxController;
        this.rumbleSide = rumbleSide;
        this.power = power;
    } 

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        switch(rumbleSide) {
            //left
            case 1:
                rumXboxController.setRumble(RumbleType.kLeftRumble, power);
            //right
            case 2:
                rumXboxController.setRumble(RumbleType.kRightRumble, power);
            //both
            case 3:
                rumXboxController.setRumble(RumbleType.kBothRumble, power);
        }
        
    }

       
    @Override
    public void end(boolean interrupted) {
        
        switch(rumbleSide) {
            //left
            case 1:
                rumXboxController.setRumble(RumbleType.kLeftRumble, 0);
            //right
            case 2:
                rumXboxController.setRumble(RumbleType.kRightRumble, 0);
            //both
            case 3:
                rumXboxController.setRumble(RumbleType.kBothRumble, 0);
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
