// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
  private WPI_TalonSRX collectorMotor;

  /** Creates a new Collector. */
  public Collector() {
    collectorMotor = new WPI_TalonSRX(8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void collect(){
    collectorMotor.set(1);
  }

  public void reverse(){
    collectorMotor.set(-1);
  }

  public void stop(){
    collectorMotor.stopMotor();
  }
}
