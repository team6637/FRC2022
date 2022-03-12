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
import frc.robot.util.linearInterpolator;

import static frc.robot.Constants.*;

public class ShooterSubsystem extends SubsystemBase {

  private final CANSparkMax motor1;
  private final CANSparkMax motor2;
  private final CANSparkMax hoodMotor;

  private final Encoder velocityEncoder = new Encoder(SHOOTER_ENCODER_1, SHOOTER_ENCODER_2);
  private final BangBangController shooterController;
  private double shooterSetpoint = 2000.0;

  private final double hoodKp = 0.4;
  private final PIDController hoodController;
  private double hoodSetpoint = 2.2;
  private final double hoodMin = 2.0;
  private final double hoodMax = 7.0;

  private double shooterPower = 0.0;

  private double [][] hoodData = {
    {4.0, 2.0},
    {7.0, 3.0},
    {10.0, 3.5},
    {16.0, 5.5}
  };

  private double [][] rpmData = {
    {4.0, 1800.0},
    {7.0, 2000.0},
    {10.0, 2200.0},
    {16.0, 2800.0}
  };

  private linearInterpolator hoodInterpolator = new linearInterpolator(hoodData);
  private linearInterpolator rpmInterpolator = new linearInterpolator(rpmData);


  private final Rev2mDistanceSensor hoodDistanceSensor;

  public ShooterSubsystem() {
    motor1 = new CANSparkMax(SHOOTER_LEFT, MotorType.kBrushless);
    motor1.restoreFactoryDefaults();

    motor2 = new CANSparkMax(SHOOTER_RIGHT, MotorType.kBrushless);
    motor2.restoreFactoryDefaults();

    hoodMotor = new CANSparkMax(HOOD, MotorType.kBrushless);
    hoodMotor.restoreFactoryDefaults();

    motor1.setInverted(false);
    motor2.setInverted(true);

    hoodMotor.restoreFactoryDefaults();
    hoodController = new PIDController(hoodKp, 0.0, 0.0);
    hoodDistanceSensor = new Rev2mDistanceSensor(Port.kOnboard);
    hoodDistanceSensor.setAutomaticMode(true);  

    SmartDashboard.putNumber("shooter target speed", shooterSetpoint);    
    SmartDashboard.putNumber("hood target", hoodSetpoint);

    shooterController = new BangBangController();
  }

  public void shoot() {
    shooterSetpoint = SmartDashboard.getNumber("shooter target speed", shooterSetpoint);
    hoodSetpoint = SmartDashboard.getNumber("hood target", hoodSetpoint);

    SmartDashboard.putNumber("shooter current speed", getSpeed());
    SmartDashboard.putBoolean("shooter is at target", atTargetVelocity());

    shooterPower = shooterController.calculate(getSpeed(), shooterSetpoint);
    motor1.set(shooterPower);
    motor2.set(shooterPower);

  }

  public void stop() {
    motor1.set(0.0);
    motor2.set(0.0);
    hoodMotor.set(0.0);
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

  public void calculateSetpointsFromDistance(double distance) {
    double distanceInFeet = distance / 12.0;
    double hoodPostion = hoodInterpolator.getInterpolatedValue(distanceInFeet);
    double speed = rpmInterpolator.getInterpolatedValue(distanceInFeet);

    SmartDashboard.putNumber("shooter target hood from limelight", hoodPostion);
    SmartDashboard.putNumber("shooter target rpm from limelight", speed);

    setSetpoints(speed, hoodPostion);
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
    setSetpoints(2600.0, 3.0);
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
    if(hoodSetpoint < hoodMin) hoodSetpoint = hoodMin;
    if(hoodSetpoint > hoodMax) hoodSetpoint = hoodMax;
    hoodMotor.set(hoodController.calculate(getHoodSensorValue(), hoodSetpoint));
    SmartDashboard.putNumber("hood current position", getHoodSensorValue());
  }
}