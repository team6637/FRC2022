// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PixySubsystem;

import java.util.ArrayList;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class PixyTargetCommand extends CommandBase {
  PixySubsystem pixySubsystem;
  Pixy2 pixy;
  private int w,h,x,y;

  public PixyTargetCommand(PixySubsystem pixySubsystem) {
    this.pixySubsystem = pixySubsystem;
    this.pixy = pixySubsystem.getPixy();

  }

  @Override
  public void initialize() {
    SmartDashboard.putBoolean("targetSeen", false);
    SmartDashboard.putNumber("targetCount", 0);
		SmartDashboard.putNumber("targetX", 0);
		SmartDashboard.putNumber("targetY", 0);
    pixy.setLamp((byte) 1, (byte) 1);
    pixy.setLED(255, 255, 255);
    this.w = pixy.getFrameWidth();
    this.h = pixy.getFrameHeight();
    SmartDashboard.putNumber("visionWidth", w);
		SmartDashboard.putNumber("visionHeight", h);

  }

  @Override
  public void execute() {
    int blockCount = pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
    SmartDashboard.putNumber("targetCount", blockCount);

    ArrayList<Block> blocks = pixy.getCCC().getBlockCache();

		if (blocks.size() <= 0) {
      SmartDashboard.putBoolean("targetSeen", false);
			return;
    }

    Block largestBlock = null;
		for (Block block : blocks) {
      if (largestBlock == null) {
        largestBlock = block;
      } else if (block.getWidth() > largestBlock.getWidth()) {
        largestBlock = block;
      }
    }
    this.x = largestBlock.getX() - w/2;
    this.y = largestBlock.getY() - h/2;
    SmartDashboard.putBoolean("targetSeen", true);
		SmartDashboard.putNumber("targetX", x);
		SmartDashboard.putNumber("targetY", y);
  }

  @Override
  public void end(boolean interrupted) {
    pixy.setLamp((byte) 0, (byte) 0);
  }

  @Override
  public boolean isFinished() { return false; }
}