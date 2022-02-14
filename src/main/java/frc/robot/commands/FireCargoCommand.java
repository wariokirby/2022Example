// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterGate;

public class FireCargoCommand extends CommandBase {
  private Shooter shooter;
  private ShooterGate gate;
  private Conveyor conveyor;
  /** Creates a new FireCargoCommand. */
  public FireCargoCommand(Shooter shooter , ShooterGate gate , Conveyor conveyor) {
    this.shooter = shooter;
    this.gate = gate;
    this.conveyor = conveyor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    addRequirements(gate);
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shootAtRange(0);
    if(Math.abs(shooter.getSetpoint() - shooter.getMeasurement()) <= 2){//TODO need to tune tolerance
      gate.open();
      conveyor.up();
    }
    else{
      gate.close();
      conveyor.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
