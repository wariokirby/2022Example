// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TargetingSystem extends SubsystemBase {
  private final double HEIGHT_OF_TARGET = 8.67;
  private final double HEIGHT_OF_CAMERA = 4;//TODO set this to mount height
  private final double MOUNT_ANGLE = 45;//TODO set this to mount angle
  private NetworkTable limeLight;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private double x;
  private double y;
  private double area;

  /** Creates a new TargetingSystem. */
  public TargetingSystem() {
    limeLight = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limeLight.getEntry("tx");
    ty = limeLight.getEntry("ty");
    ta = limeLight.getEntry("ta");
    x = 100;
    y = 100;
    area = -1;
    limeLight.getEntry("ledMode").setNumber(1);
    limeLight.getEntry("camMode").setNumber(0);
    limeLight.getEntry("pipeline").setNumber(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    x = tx.getDouble(0);
    y = ty.getDouble(0);
    area = ta.getDouble(0);
    SmartDashboard.putNumber("Limelight X", x);
    SmartDashboard.putNumber("Limelight Y", y);
    SmartDashboard.putNumber("Limelight Area", area);

  }

  public double getTargetX(){
    return x;
  }
  public double getTargetY(){
    return y;
  }
  public double getTargetArea(){
    return area;
  }

  public void ledControl(boolean on){
    NetworkTableEntry led = limeLight.getEntry("ledMode");
    if(on){
      led.setNumber(3);
    }
    else{
      led.setNumber(1);
    }
  }

  public double calcRange(){
    double d = (HEIGHT_OF_TARGET - HEIGHT_OF_CAMERA) / Math.tan(Math.toRadians(MOUNT_ANGLE + y));
    return d;
  }
 
}
