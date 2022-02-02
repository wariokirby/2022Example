// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class PIDMotorController extends PIDSubsystem implements MotorController{
  private MotorController motorController;
  private Encoder encoder;
  private SimpleMotorFeedforward driveFF;


  public static final double DRIVE_SPEED_MAX = 12.0;//TODO need to measure this
  public final double DRIVE_FEEDFORWARD_KS = .1;//volts necessary to barely start the wheel rotating
  public final double DRIVE_FEEDFORWARD_KV = 12.0 / DRIVE_SPEED_MAX; //volts to maintain a speed, calc based on 12V to maintain max speed
  /** Creates a new PIDMotorController. */
  public PIDMotorController(MotorController motorController , Encoder encoder) {
    super(
        // The PIDController used by the subsystem
        new PIDController(1, 0, 0));
    this.motorController = motorController;
    this.encoder = encoder;

    driveFF = new SimpleMotorFeedforward(DRIVE_FEEDFORWARD_KS, DRIVE_FEEDFORWARD_KV);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    motorController.setVoltage(output + driveFF.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return encoder.getRate();
    
  }

  public void set(double speed){
    //motorController.set(speed);
    setSetpoint(speed * DRIVE_SPEED_MAX);
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
