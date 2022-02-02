// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTracker;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.*;

public class AcquireCargoCommand extends CommandBase {
  private Drivetrain drivetrain;
  private CargoTracker tracker;
  private int[] trackerData;
  private final int SIZE_WHEN_CAUGHT = 20;//TODO need to measure this


  /** Creates a new AcquireCargo. */
  public AcquireCargoCommand(Drivetrain drivetrain , CargoTracker tracker) {
    this.drivetrain = drivetrain;
    this.tracker = tracker;
    trackerData = new int[2];

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    addRequirements(tracker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trackerData = tracker.findClosestCargo();
    if(trackerData[0] == 160){
      drivetrain.drive(0, .5);
    }
    else if(Math.abs(trackerData[0]) > 2){//allow 2 degrees each direction, needs to be tuned
      drivetrain.drive(1 * (1-(trackerData[1]/SIZE_WHEN_CAUGHT)) + MIN_DRIVE_POWER, 1 * (trackerData[0] / 157));//TODO need to tune gain values
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(trackerData[1] >= SIZE_WHEN_CAUGHT){
      return true;
    }
    return false;
  }
}
