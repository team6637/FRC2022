// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax motor1;
  CANSparkMax motor2;
  CANSparkMax hoodmotor;

  Encoder velocityEncoder = new Encoder(SHOOTER_ENCODER_1, SHOOTER_ENCODER_2);
  BangBangController shooterController = new BangBangController();
  double shooterSetpoint = 2000.0;

  double hoodKp = 0.4;
  PIDController hoodController;
  double hoodSetpoint = 2.2;
  double hoodMin = 2.0;
  double hoodMax = 8.0;

  Rev2mDistanceSensor hoodDistanceSensor;

  public ShooterSubsystem() {
    motor1 = new CANSparkMax(SHOOTER_LEFT, MotorType.kBrushless);
    motor1.restoreFactoryDefaults();

    motor2 = new CANSparkMax(SHOOTER_RIGHT, MotorType.kBrushless);
    motor2.restoreFactoryDefaults();

    hoodmotor = new CANSparkMax(HOOD, MotorType.kBrushless);
    hoodmotor.restoreFactoryDefaults();

    motor1.setInverted(false);
    motor2.setInverted(true);

    hoodmotor.restoreFactoryDefaults();
    hoodController = new PIDController(hoodKp, 0.0, 0.0);
    hoodDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    hoodDistanceSensor.setAutomaticMode(true);  

    SmartDashboard.putNumber("shooter target speed", shooterSetpoint);    
    SmartDashboard.putNumber("hood target", hoodSetpoint);

    shooterController = new BangBangController();
    //shooterController.setTolerance(40);

  }

  public void shoot() {
    shooterSetpoint = SmartDashboard.getNumber("shooter target speed", shooterSetpoint);

    double power = shooterController.calculate(getSpeed(), shooterSetpoint);
    motor1.set(power);
    motor2.set(power);
  }

  public void stop() {
    motor1.set(0.0);
    motor2.set(0.0);
  }

  public double getSpeed() {
    return velocityEncoder.getRate() / 2048.0 * 60.0;
  }

  public void setSetpoints(double s, double h) {
    SmartDashboard.putNumber("shooter target speed", s);
    SmartDashboard.putNumber("hood target", h);
    shooterSetpoint = s;
    hoodSetpoint = h;
  }


  public void setLowClose() {
    setSetpoints(700.0, 4.0);
  }

  public void setLowFar() {
    setSetpoints(1200.0, 5.0);
  }


  public void setHighClose() {
    setSetpoints(2200.0, 2.0);
  }

  public void setHighFar() {
    setSetpoints(5100.0, 3.0);
  }

  public void setHighAuto() {
    
    setSetpoints(2300.0, 2.0);

  }

  public double getHoodSensorValue() {
    double v = hoodDistanceSensor.getRange();
    if(v < hoodMin) v = hoodMin;
    return v;
  }

  public boolean atTargetVelocity() {
    return getSpeed() > shooterSetpoint;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("shooter speed", getSpeed());
    //SmartDashboard.putNumber("hood distance sensor", getHoodSensorValue());

    hoodSetpoint = SmartDashboard.getNumber("hood target", hoodSetpoint);

    if(hoodSetpoint < hoodMin) hoodSetpoint = hoodMin;
    if(hoodSetpoint > hoodMax) hoodSetpoint = hoodMax;
    hoodmotor.set(hoodController.calculate(getHoodSensorValue(), hoodSetpoint));


    SmartDashboard.putBoolean("shooter is at target", atTargetVelocity());

  }
}