// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class CargoTracker extends SubsystemBase {
  private Pixy2 pixy;
  private int blockCount;
  private int signature;
  /** Creates a new CargoTracker. */
  public CargoTracker(boolean color) {//color true is sig 1, false is sig 2
    if(color){
      signature = Pixy2CCC.CCC_SIG1;
    }
    else{
      signature = Pixy2CCC.CCC_SIG2;
    }
    pixy = Pixy2.createInstance(new SPILink());
    pixy.init();

  }

  @Override
  public void periodic() {
    blockCount = pixy.getCCC().getBlocks(false , signature , 11);
    SmartDashboard.putNumber("Targets Found", blockCount);
    
    // This method will be called once per scheduler run
  }

  public int[] findClosestCargo(){//returns how far off center in x direction
    int[] dirSize = new int[2];// 0 is direction to center of block, 1 is width of block
    if(blockCount <= 0){
      dirSize[0] = 200;
      return dirSize;
    }
    ArrayList<Block> foundCargo = pixy.getCCC().getBlockCache();
    Block closestCargo = null;
    for(Block cargo : foundCargo){
      if(closestCargo == null){
        closestCargo = cargo;
      }
      else if(cargo.getWidth() > closestCargo.getWidth()){
        closestCargo = cargo;
      }
    }//end for

    dirSize[0] = closestCargo.getX() -  157;
    dirSize[1] = closestCargo.getWidth();
    return dirSize; 
  }//end findClosestCargo
}//end class
