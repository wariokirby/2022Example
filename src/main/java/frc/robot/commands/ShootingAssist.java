// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TargetingSystem;
import static frc.robot.Constants.*;

public class ShootingAssist extends CommandBase {
  private Drivetrain drivetrain;
  private TargetingSystem targetingSystem;
  private Shooter shooter;
  /** Creates a new ShootingAssist. */
  public ShootingAssist(Drivetrain drivetrain , TargetingSystem targetingSystem , Shooter shooter) {
    this.drivetrain = drivetrain;
    this.targetingSystem = targetingSystem;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    addRequirements(targetingSystem);
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetingSystem.ledControl(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
//this is for a fixed range shooter
    if(Math.abs(targetingSystem.getTargetX()) < 1 && Math.abs(targetingSystem.distanceToFixedRange()) < .25){//3 inches in either direction
      drivetrain.stop();
      shooter.shootAtRange(0);//if used, this argument would go away
    }//end if
    else{
      drivetrain.drive((targetingSystem.distanceToFixedRange() / 5) + MIN_DRIVE_POWER, 1 * (targetingSystem.getTargetX()/27.0) + MIN_TURN_POWER);
      //5 should be tuned to get the quickest usable reponse for moving to range
    }//end else

//This is in case we have a variable range shooter
    /*if(Math.abs(targetingSystem.getTargetX()) < 1){
      drivetrain.stop();
      shooter.shootAtRange(targetingSystem.calcRange());
    }
    else{
      drivetrain.drive(0, 1 * (targetingSystem.getTargetX()/27.0) + MIN_TURN_POWER);
    }
  }*/

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    targetingSystem.ledControl(false);
    shooter.manualOverride(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(targetingSystem.getTargetArea() == 0){
      return true;
    }
    return false;
  }
}
