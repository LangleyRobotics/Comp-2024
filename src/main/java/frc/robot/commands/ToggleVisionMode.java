package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CameraSubsystem;

public class ToggleVisionMode extends Command{
    private final CameraSubsystem cam;
    public ToggleVisionMode(CameraSubsystem cam) {
        this.cam = cam;
    }
    
    @Override
    public void initialize() {
        //cam.setModeVision();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
    
}
