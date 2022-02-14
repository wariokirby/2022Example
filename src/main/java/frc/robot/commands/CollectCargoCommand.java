// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CargoTracker;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterGate;

public class CollectCargoCommand extends CommandBase {
  private Drivetrain drivetrain;
  private CargoTracker tracker;
  private Collector collector;
  private Conveyor conveyor;
  private ShooterGate gate;

  private int[] trackerData;
  private final int MIN_SIZE_WHEN_PRESENT = 18;//TODO need to measure this

  /** Creates a new CollectCargoCommand. */
  public CollectCargoCommand(Drivetrain drivetrain , CargoTracker tracker , Collector collector , Conveyor conveyor , ShooterGate gate) {
    this.drivetrain = drivetrain;
    this.tracker = tracker;
    this.collector = collector;
    this.conveyor = conveyor;
    this.gate = gate;

    trackerData = new int[2];

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    addRequirements(tracker);
    addRequirements(collector);
    addRequirements(conveyor);
    addRequirements(gate);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gate.close();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trackerData = tracker.findClosestCargo();
    collector.collect();
    conveyor.up();
    drivetrain.drive(.25, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(trackerData[1] < MIN_SIZE_WHEN_PRESENT){
      return true;
    }
    return false;
  }
}
