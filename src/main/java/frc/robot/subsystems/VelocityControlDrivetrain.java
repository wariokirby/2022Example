// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VelocityControlDrivetrain extends SubsystemBase {
  private final int LEFT_DRIVE_ENCODER_CHANNEL_A = 0;
  private final int LEFT_DRIVE_ENCODER_CHANNEL_B = 1;
  private final int RIGHT_DRIVE_ENCODER_CHANNEL_A = 2;
  private final int RIGHT_DRIVE_ENCODER_CHANNEL_B = 3;
  private final double WHEEL_RADIUS_FEET = 2.0 / 12;


  private DifferentialDrive robotDrive;
  //private MotorControllerGroup leftDrive;
  //private MotorControllerGroup rightDrive;

  private PIDMotorController leftPIDDrive;
  private PIDMotorController rightPIDDrive;

  private Encoder leftDriveEncoder;
  private Encoder rightDriveEncoder;

  /** Creates a new Drivetrain. */
  public VelocityControlDrivetrain() {
    var leftFront = new WPI_TalonSRX(1);
		var leftBack = new WPI_TalonSRX(2);
		var leftTop = new WPI_TalonSRX(3);
		leftTop.setInverted(true);
    var leftDrive = new MotorControllerGroup(leftFront, leftTop , leftBack);

    leftDriveEncoder = new Encoder(LEFT_DRIVE_ENCODER_CHANNEL_A, LEFT_DRIVE_ENCODER_CHANNEL_B, false, EncodingType.k4X);
		leftDriveEncoder.setDistancePerPulse(Math.PI * 2 * WHEEL_RADIUS_FEET / 2048.0);
    leftPIDDrive = new PIDMotorController(leftDrive, leftDriveEncoder);


    var rightFront = new WPI_TalonSRX(4);
		var rightBack = new WPI_TalonSRX(5);
		var rightTop = new WPI_TalonSRX(6);
    var rightDrive = new MotorControllerGroup(rightFront , rightTop , rightBack);
    rightDrive.setInverted(true);

    rightDriveEncoder = new Encoder(RIGHT_DRIVE_ENCODER_CHANNEL_A, RIGHT_DRIVE_ENCODER_CHANNEL_B, false, EncodingType.k4X);
		rightDriveEncoder.setDistancePerPulse(Math.PI * 2 * WHEEL_RADIUS_FEET / 2048.0);
    rightPIDDrive = new PIDMotorController(rightDrive, rightDriveEncoder);

    robotDrive = new DifferentialDrive(leftPIDDrive , rightPIDDrive);
	
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Encoder Velocity", leftDriveEncoder.getRate());
		SmartDashboard.putNumber("Right Encoder Velocity", rightDriveEncoder.getRate());
		SmartDashboard.putNumber("Left Encoder Distance", leftDriveEncoder.getDistance());
		SmartDashboard.putNumber("Right Encoder Distance", rightDriveEncoder.getDistance());
    // This method will be called once per scheduler run
  }

  public void drive(double throttle , double steer){
    robotDrive.arcadeDrive(throttle, steer);
  }

  public double getLeftEncoderDistance() {
		return leftDriveEncoder.getDistance();
	}

	public double getLeftRate(){
		return leftDriveEncoder.getRate();
	}

	public double getRightRate(){
		return rightDriveEncoder.getRate();
	}

	public double getRightEncoderDistance() {
		return rightDriveEncoder.getDistance();
	}

  public void stop(){
    leftPIDDrive.stopMotor();
    rightPIDDrive.stopMotor();
  }
}
