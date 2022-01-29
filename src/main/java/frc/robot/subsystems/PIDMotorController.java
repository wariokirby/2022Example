// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDMotorController extends PIDSubsystem implements MotorController{
  private MotorController motorController;
  private Encoder encoder;
  /** Creates a new PIDMotorController. */
  public PIDMotorController(MotorController motorController , Encoder encoder) {
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
    this.motorController = motorController;
    this.encoder = encoder;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }

  public void set(double speed){
    motorController.set(speed);
  }
  public double get(){
    return motorController.get();
  }
  public void setInverted(boolean isInverted){
    motorController.setInverted(isInverted);
  }
  public boolean getInverted(){
    return motorController.getInverted();
  }
  public void stopMotor(){
    motorController.stopMotor();
  }
}
