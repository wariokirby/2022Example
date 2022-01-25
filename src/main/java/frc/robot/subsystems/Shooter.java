// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Shooter extends PIDSubsystem {
  private final int SHOOTER_ENCODER_CHANNEL_A = 4;
  private final int SHOOTER_ENCODER_CHANNEL_B = 5;
  //following is based on 2020 shooter, will have to modify for 2022
  public static final double SHOOTER_RPS_MAX = 3000.0 / 60.0;
  public final double SHOOTER_FEEDFORWARD_KS = .05;//volts necessary to barely start the wheel rotating
  public final double SHOOTER_FEEDFORWARD_KV = 12.0 / SHOOTER_RPS_MAX; //volts to maintain a speed, calc based on 12V to maintain max speed

  private WPI_TalonSRX motors;
  private Encoder shooterEncoder;
  private SimpleMotorFeedforward shooterFF;

  /** Creates a new Shooter. */
  public Shooter() {
    super(
        // The PIDController used by the subsystem
        new PIDController(1, 0, 0));

    motors = new WPI_TalonSRX(7);
    shooterEncoder = new Encoder(SHOOTER_ENCODER_CHANNEL_A, SHOOTER_ENCODER_CHANNEL_B, true, EncodingType.k4X);
    shooterEncoder.setDistancePerPulse(1.0 / 2048.0);//set for rotations per second

    shooterFF = new SimpleMotorFeedforward(SHOOTER_FEEDFORWARD_KS, SHOOTER_FEEDFORWARD_KV);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    motors.setVoltage(output + shooterFF.calculate(setpoint));
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return shooterEncoder.getRate();

  }

  public void manualOverride(double throttle){
    SmartDashboard.putNumber("Shooter RPM", shooterEncoder.getRate() * 60.0);
    if(Math.abs(throttle) > .1){
       motors.set(throttle);
    }//end deadband adjustment
   
  }

  public void autoShooter(double targetFPS){
    SmartDashboard.putNumber("Shooter RPM", shooterEncoder.getRate() * 60.0);
    setSetpoint(targetFPS);
  }

  public Encoder getEncoder(){
    return shooterEncoder;
  }
}
