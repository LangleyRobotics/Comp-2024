// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;

//PathPlanner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.FollowPathCommand;

//Constants Imports 
import frc.robot.Constants.AutoConstants;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Buttons;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AllForNaught;
//import frc.robot.Trajectories;

//Command Imports
import frc.robot.commands.DriveToPointCmd;
import frc.robot.commands.RumbleCmd;
import frc.robot.commands.OTFTrajectoryFactory;
import frc.robot.commands.SwerveControllerCmd;
import frc.robot.commands.ShooterControllerCmd;
import frc.robot.commands.PivotControllerCmd;
import frc.robot.commands.ResetPivotCmd;
import frc.robot.commands.ClimbAutoCmd;
import frc.robot.commands.ClimbControllerCmd;
import frc.robot.commands.ClimbSwitchDirCmd;
import frc.robot.commands.SetPivotCmd;
import frc.robot.commands.AutoAlignShootCmd;
import frc.robot.commands.AutoAlignAmpCmd;
import frc.robot.commands.IntakeCmd;

//Subsystem Imports
import frc.robot.subsystems.*;

//Auto Commands
import frc.robot.commands.ShootAutoCmd;
import frc.robot.commands.IntakeAutoCmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import edu.wpi.first.wpilibj.util.Color;



/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();


  XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController operatorController = new XboxController(OIConstants.kSecondaryControllerPort);
  
  SendableChooser<Command> autoChooser = new SendableChooser<>();






  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //TEST Auto align the wheels before matches
    //robotDrive.initModulesReset(false);
  
    // Configure default commands

     
    robotDrive.setDefaultCommand(
        new SwerveControllerCmd(
            robotDrive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> true, // field oriented
            () -> false));

    shooterSubsystem.setDefaultCommand(
      new ShooterControllerCmd(
        shooterSubsystem,
        () -> operatorController.getLeftBumper(),
        () -> operatorController.getRightBumper(),
        () -> false));

    intakeSubsystem.setDefaultCommand(
      new IntakeCmd(
        intakeSubsystem, 
        () -> 0.0, 
        0,
        false));

    pivotSubsystem.setDefaultCommand(
      new PivotControllerCmd(
        pivotSubsystem, 
        () -> operatorController.getRightTriggerAxis(),
        () -> operatorController.getLeftTriggerAxis()));

    climbSubsystem.setDefaultCommand(
      new ClimbControllerCmd(
        climbSubsystem, 
        () -> false, 
        false,
        "none"));

    // robotDrive.zeroHeading();
    //robotDrive.initModulesReset();

    //lightingSubsystem.setDefaultCommand(lightingSubsystem.splitColor(Color.kAquamarine, Color.kDarkCyan));



    // Register Named Commands
    //Named commands = commands other than driving around that still need to be executed in auto

    var pivotToIntake = new SetPivotCmd(pivotSubsystem, 0).withTimeout(1.5);
    var pivotToShootUpClose = new SetPivotCmd(pivotSubsystem, 1).withTimeout(2);
    var pivotToShootR1 = new SetPivotCmd(pivotSubsystem, 3).withTimeout(1.5);
    var pivotToShootSide = new SetPivotCmd(pivotSubsystem, 4).withTimeout(2);
    var pivotToShoot301 = new SetPivotCmd(pivotSubsystem, 5).withTimeout(1.5);

    var intake = new IntakeAutoCmd(intakeSubsystem, -1, false).withTimeout(2.5);
    var TESTgroundIntake = new SequentialCommandGroup(
      new IntakeAutoCmd(intakeSubsystem, -1, false).withTimeout(2.5),
      new ParallelCommandGroup(
        new IntakeAutoCmd(intakeSubsystem, 1, false).withTimeout(0.4),
        new ShootAutoCmd(shooterSubsystem, -1).withTimeout(0.4)));
    var groundIntake = new IntakeAutoCmd(intakeSubsystem, -1, true).withTimeout(2.5);

    var leftTelescopeDown = new ClimbAutoCmd(climbSubsystem, false, "left").withTimeout(0.3);


    var shoot = new ParallelCommandGroup(
      new ShootAutoCmd(shooterSubsystem, 1).withTimeout(2),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new IntakeAutoCmd(intakeSubsystem, -1, false).withTimeout(1)));

    //var shooterOff=new ShootAutoCmd(shooterSubsystem, 0).withTimeout(0.01);
    
    var pivotToAlignShoot = new ParallelCommandGroup(
      new AutoAlignShootCmd(limelightSubsystem, pivotSubsystem, shooterSubsystem).withTimeout(2),
      new SequentialCommandGroup(
        new WaitCommand(1),
        new IntakeAutoCmd(intakeSubsystem, -1, false).withTimeout(1)));


    //Named Commands for PathPlanner
    NamedCommands.registerCommand("Pivot To Intake", pivotToIntake);
    NamedCommands.registerCommand("Pivot To Shoot Up Close", pivotToShootUpClose);
    NamedCommands.registerCommand("Pivot To Shoot Side", pivotToShootSide);
    NamedCommands.registerCommand("Pivot To Align And Shoot", pivotToAlignShoot);
    NamedCommands.registerCommand("Pivot To Shoot 3.1.1", pivotToShootR1);
    NamedCommands.registerCommand("Pivot To Shoot 3.0.1", pivotToShoot301);
    NamedCommands.registerCommand("Shoot", shoot);
    //NamedCommands.registerCommand("Shooter Off", shooterOff);
    NamedCommands.registerCommand("Intake", intake);
    NamedCommands.registerCommand("TEST Ground Intake", TESTgroundIntake);
    NamedCommands.registerCommand("Ground Intake", groundIntake);
    NamedCommands.registerCommand("Left Telescope Down", leftTelescopeDown);


    // Configure the button bindings
    configureButtonBindings();

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // SmartDashboard.putData("Straight Auto", new PathPlannerAuto("Straight Auto"));
    // SmartDashboard.putData("ShootAndStraight Auto", new PathPlannerAuto("ShootAndStraight Auto"));
    //SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));



    
    
    //Intake ring
    new JoystickButton(operatorController, Buttons.X).whileTrue(new IntakeCmd(intakeSubsystem, 
      () -> IntakeConstants.kIntakeMotorSpeed, -1, false));

    //Outtake ring
    new JoystickButton(operatorController, Buttons.B).whileTrue(new IntakeCmd(intakeSubsystem, 
      () -> IntakeConstants.kIntakeMotorSpeed, 1, false));

    //Ground intake (limited)
    new JoystickButton(operatorController, Buttons.A).whileTrue(new IntakeCmd(intakeSubsystem, 
    () -> IntakeConstants.kIntakeMotorSpeed, -1, true));



    //Shoot out ring
    new JoystickButton(operatorController, Buttons.RB).whileTrue(new ShooterControllerCmd(shooterSubsystem, 
    () -> true, () -> false, () -> false));

    //Shoot in ring
    new JoystickButton(operatorController, Buttons.LB).whileTrue(new ShooterControllerCmd(shooterSubsystem, 
    () -> false, () -> true, () -> false));

    //Shoot ferry
    var ferryCmd = new ParallelCommandGroup(
      new ShooterControllerCmd(shooterSubsystem, () -> false, () -> false, () -> true).withTimeout(1.1),
      new SequentialCommandGroup(
        new WaitCommand(0.8),
        new IntakeCmd(intakeSubsystem, () -> IntakeConstants.kIntakeMotorSpeed, -1, false).withTimeout(0.3)));
    
    new JoystickButton(operatorController, Buttons.Y).whileTrue(ferryCmd);



    //Set pivot position to intake
    new POVButton(operatorController, Buttons.DOWN_ARR).whileTrue(new SetPivotCmd(pivotSubsystem, 0));
    
    //Set pivot position to shoot up close
    new POVButton(operatorController, Buttons.RIGHT_ARR).whileTrue(new SetPivotCmd(pivotSubsystem, 1));

    //Set pivot position to amp scoring
    new POVButton(operatorController, Buttons.UP_ARR).whileTrue(new SetPivotCmd(pivotSubsystem, 2));


    //Autoalign pivot
    new POVButton(operatorController, Buttons.LEFT_ARR).whileTrue(new AutoAlignShootCmd(limelightSubsystem, pivotSubsystem, shooterSubsystem));

    //Autoalign amp
    new POVButton(driverController, Buttons.LEFT_ARR).whileTrue(new AutoAlignAmpCmd(robotDrive, limelightSubsystem));



    //Climb expand right
    new JoystickButton(driverController, Buttons.Y).whileTrue(new ClimbControllerCmd(climbSubsystem, () -> true, true, "right"));
    
    //Climb collapse right
    new JoystickButton(driverController, Buttons.A).whileTrue(new ClimbControllerCmd(climbSubsystem, () -> true, false, "right"));

    //Climb expand left
    new POVButton(driverController, Buttons.UP_ARR).whileTrue(new ClimbControllerCmd(climbSubsystem, () -> true, true, "left"));
    
    //Climb collapse left
    new POVButton(driverController, Buttons.DOWN_ARR).whileTrue(new ClimbControllerCmd(climbSubsystem, () -> true, false, "left"));

    //Climb expand both
    new JoystickButton(driverController, Buttons.RB).whileTrue(new ClimbControllerCmd(climbSubsystem, () -> true, true, "both"));

    //Climb collapse both
    new JoystickButton(driverController, Buttons.LB).whileTrue(new ClimbControllerCmd(climbSubsystem, () -> true, false, "both"));

    //Climb switch right direction
    new JoystickButton(driverController, Buttons.Menu).whileTrue( new ClimbSwitchDirCmd(climbSubsystem,'r'));

    //Climb switch left direction
    new JoystickButton(driverController, Buttons.Maria).whileTrue(new ClimbSwitchDirCmd(climbSubsystem, 'l'));



    
    //Reset pivot encoder at illegal position
    new JoystickButton(operatorController, Buttons.Maria).whileTrue(new ResetPivotCmd(pivotSubsystem));



    //Rumble controllers
    //new JoystickButton(driverController, Buttons.Maria).whileTrue(new RumbleCmd(operatorController, 1, 1.00));
    new JoystickButton(operatorController, Buttons.L3).whileTrue(new RumbleCmd(driverController, 1, 1.00));
    new JoystickButton(operatorController, Buttons.R3).whileTrue(new RumbleCmd(driverController, 2, 1.00));



    //Zero Heading
    new JoystickButton(driverController, Buttons.X).toggleOnTrue(new AllForNaught(robotDrive));

    //Slow drive with d-pad
    // new POVButton(driverController, Buttons.DOWN_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> true,  () -> false));
    // new POVButton(driverController, Buttons.UP_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> 0.0, () -> true,  () -> false));
    // new POVButton(driverController, Buttons.RIGHT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> -DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> true,  () -> false));
    // new POVButton(driverController, Buttons.LEFT_ARR).whileTrue(new SwerveControllerCmd(robotDrive, () -> 0.0, () -> DriveConstants.kSlowDriveCoefficient, () -> 0.0, () -> true,  () -> false));



    new JoystickButton(driverController, Buttons.B).onTrue(new InstantCommand(() -> robotDrive.resetEncoders()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return the autonomous command given by the drop-down selector in ShuffleBoard
    return autoChooser.getSelected();
  }

}
