// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//test
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AcquireCargoCommand;
import frc.robot.commands.CollectCargoCommand;
import frc.robot.commands.DriveForwardCommand;
import frc.robot.commands.FireCargoCommand;
import frc.robot.commands.ShootingAssist;
import frc.robot.subsystems.CargoTracker;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shifter;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterGate;
import frc.robot.subsystems.TargetingSystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Joystick throttle = new Joystick(0);
  private final Joystick wheel = new Joystick(1);
  private final XboxController xBox = new XboxController(2);
  private final Joystick prajBox = new Joystick(3);
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Shifter shifter = new Shifter(drivetrain);
  private final Shooter shooter = new Shooter();
  private final Collector collector = new Collector();
  private final Conveyor conveyor = new Conveyor();
  private final ShooterGate gate = new ShooterGate();
  private final CargoTracker tracker = new CargoTracker(true);
  private final TargetingSystem targetingSystem = new TargetingSystem();


  private final DriveForwardCommand driveForwardCommand = new DriveForwardCommand(drivetrain);
  private final SequentialCommandGroup findBallAndShootCommand = new SequentialCommandGroup(
    new AcquireCargoCommand(drivetrain , tracker) , 
    new CollectCargoCommand(drivetrain, tracker, collector, conveyor, gate) , 
    new ShootingAssist(drivetrain, targetingSystem, shooter, gate) , 
    new FireCargoCommand(shooter, gate, conveyor)
    );

    SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoChooser.setDefaultOption("Drive Forward", driveForwardCommand);
    autoChooser.addOption("Find and Shoot", findBallAndShootCommand);

    SmartDashboard.putData(autoChooser);

    SmartDashboard.putData(drivetrain);
    SmartDashboard.putData(tracker);
    SmartDashboard.putData(collector);
    SmartDashboard.putData(conveyor);
    SmartDashboard.putData(shooter);
    SmartDashboard.putData(gate);
    SmartDashboard.putData(targetingSystem);

    drivetrain.setDefaultCommand(new RunCommand(
      () -> drivetrain.drive(-throttle.getY() , wheel.getX()), 
      drivetrain
    ));

    shifter.setDefaultCommand(new RunCommand(
      () -> shifter.autoShift(),
       shifter
    ));

    shooter.setDefaultCommand(new RunCommand(
      () -> shooter.manualOverride(xBox.getRightTriggerAxis()), 
      shooter
    ));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(throttle, 2).whenHeld(new AcquireCargoCommand(drivetrain, tracker));//activate track and get cargo

    new JoystickButton(throttle, 1).whenHeld(new ShootingAssist(drivetrain, targetingSystem, shooter , gate));//activate aim and spin shooter

    new JoystickButton(throttle, 3).whenPressed(new InstantCommand(drivetrain::stop , drivetrain));//stop the robot and shut down anything driving it other than the pilot

    new JoystickButton(xBox, Button.kA.value).whenPressed(new RunCommand(collector::collect, collector));

    new JoystickButton(xBox, Button.kB.value).whenPressed(new RunCommand(collector::reverse, collector));

    new JoystickButton(xBox, Button.kX.value).whenPressed(new InstantCommand(collector::stop, collector));

    new POVButton(xBox, 180)
      .whenPressed(new InstantCommand(gate::open , gate))
      .whenReleased(new InstantCommand(gate::close , gate));

    new JoystickButton(xBox, Button.kRightBumper.value)
      .whenPressed(new RunCommand(conveyor::up , conveyor))
      .whenReleased(new InstantCommand(conveyor::stop , conveyor));

    new JoystickButton(xBox, Button.kLeftBumper.value)
      .whenPressed(new RunCommand(conveyor::down , conveyor))
      .whenReleased(new InstantCommand(conveyor::stop , conveyor));


    new JoystickButton(prajBox, 2).whenHeld(new StartEndCommand(
      () -> shifter.setGear(true), 
      shifter::autoShift, 
      shifter
      ));//manual set high gear

    new JoystickButton(prajBox, 3).whenHeld(new StartEndCommand(
      () -> shifter.setGear(false), 
      shifter::autoShift, 
      shifter
      ));//manual set low gear
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
