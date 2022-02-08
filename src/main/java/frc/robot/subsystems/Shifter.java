// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shifter extends SubsystemBase {
  private DoubleSolenoid shifter;
  private Drivetrain drivetrain;

  private final int HIGH_GEAR_CHANNEL = 0;//TODO need to match robot
  private final int LOW_GEAR_CHANNEL = 1;

  private final double SHIFT_POINT_HIGH = 4.5;//in ft/s TODO need to tune these to actual performance
  private final double SHIFT_POINT_LOW = 4;


  /** Creates a new Shifter. */
  public Shifter(Drivetrain drivetrain) {
    shifter = new DoubleSolenoid(PneumaticsModuleType.CTREPCM , HIGH_GEAR_CHANNEL , LOW_GEAR_CHANNEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("High Gear", isHighGear());
  }

  public boolean isHighGear(){
    return shifter.get() == Value.kForward;
  }//end isHighGear

  public void setGear(boolean isHigh){
    if(isHigh){
      shifter.set(Value.kForward);
    }//end if
    else{
      shifter.set(Value.kReverse);
    }//end else
  }//end setgear

  public void autoShift(){
    //low to high
    if(!isHighGear() && (Math.abs(drivetrain.getLeftRate()) > SHIFT_POINT_HIGH || Math.abs(drivetrain.getRightRate()) > SHIFT_POINT_HIGH)){
      setGear(true);
    }//end if
    //high to low
    else if(isHighGear() && (Math.abs(drivetrain.getLeftRate()) < SHIFT_POINT_LOW || Math.abs(drivetrain.getRightRate()) < SHIFT_POINT_LOW)){
      setGear(false);
    }//end else if 
  }//end autoShift

}//end subsystem
