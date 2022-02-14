// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  CANSparkMax motor = new CANSparkMax(54, MotorType.kBrushless);
  Rev2mDistanceSensor distanceOnBoard;

  public IntakeSubsystem() {
    distanceOnBoard = new Rev2mDistanceSensor(Port.kOnboard);
    distanceOnBoard.setAutomaticMode(true);  
  }

  public void in() {
    motor.set(-0.6);
  }

  public void out() {
    motor.set(0.6);
  }

  public void stop() {
    motor.set(0.0);
  }

  public boolean ballIsSeen() {
    if(distanceOnBoard.getRange() < 6) {
      return true;
    } else {
      return false;
    }
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("intake distance sensor", distanceOnBoard.getRange());
  }
}
