// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class LiftSubsystem extends SubsystemBase {

    CANSparkMax motor1 = new CANSparkMax(LIFT_LEFT, MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(LIFT_RIGHT, MotorType.kBrushless);
    DutyCycleEncoder leftEncoder = new DutyCycleEncoder(LIFT_LEFT_ENCODER);
    DutyCycleEncoder rightEncoder = new DutyCycleEncoder(LIFT_RIGHT_ENCODER);

    double liftTarget = 0.0;
    double liftTargetMin = 0.0;
    double liftTargetMax = 6.0;
    boolean liftLeftWasReset = false;
    boolean liftRightWasReset = false;
    PIDController liftPID;
    double liftKp = 0.8;

    CANSparkMax winch = new CANSparkMax(WINCH, MotorType.kBrushless);

    double winchSetpoint = 0.0;
    double winchSetpointMin = 0.0;
    double winchSetpointMax = 100;

    private RelativeEncoder winchEncoder;

    PIDController winchPID;
    double winchKp = 0.2;

    public LiftSubsystem() {

      leftEncoder.setConnectedFrequencyThreshold(900);
      rightEncoder.setConnectedFrequencyThreshold(900);
      liftPID = new PIDController(liftKp, 0.0, 0.0);

      winchPID = new PIDController(winchKp, 0.0, 0.0);
      winchEncoder = winch.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    }

    public void resetLift() {
        leftEncoder.reset();
        rightEncoder.reset();
        liftTarget = 0;        
    }

    public void up() {
        liftTarget += 0.1;
    }

    public void down() {
        liftTarget -= 0.1;
    }

    public void in() {
        winchSetpoint -= 1.0;
    }

    public void out() {
        winchSetpoint += 1.0;
    }

    public double getWinchPosition() {
        return winchEncoder.getPosition();
    }

    public void setWinchSetPoint(double s) {
        winchSetpoint = s;
    }

    public boolean isWinchAtSetpoint() {
        return winchPID.atSetpoint();
    }


    @Override
    public void periodic() {

        if(liftLeftWasReset == false && leftEncoder.get() != 0.0) {
            resetLift();
            liftLeftWasReset = true;
        }

        if(liftRightWasReset == false && rightEncoder.get() != 0.0) {
            resetLift();
            liftRightWasReset = true;
        }


        if(liftTarget > liftTargetMax) liftTarget = liftTargetMax;
        if(liftTarget < liftTargetMin) liftTarget = liftTargetMin;

        if(winchSetpoint > winchSetpointMax) winchSetpoint = winchSetpointMax;
        if(winchSetpoint < winchSetpointMin) winchSetpoint = winchSetpointMin;

        SmartDashboard.putNumber("lift left encoder position", leftEncoder.get());
        SmartDashboard.putNumber("lift right encoder position", rightEncoder.get());
        SmartDashboard.putNumber("lift target", liftTarget);

        SmartDashboard.putNumber("lift winch position", getWinchPosition());
        SmartDashboard.putNumber("lift winch setpoint", winchSetpoint);

        double leftPower = liftPID.calculate(leftEncoder.get(), liftTarget);
        double rightPower = liftPID.calculate(rightEncoder.get(), liftTarget);
        double winchPower = winchPID.calculate(getWinchPosition(), winchSetpoint);

        motor1.set(leftPower);
        motor2.set(rightPower);
        winch.set(winchPower);

        SmartDashboard.putNumber("lift left power", leftPower);
        SmartDashboard.putNumber("lift right power", rightPower);
        SmartDashboard.putNumber("winch power", winchPower);
        SmartDashboard.putNumber("winch encoder", winchEncoder.getPosition());
        
        SmartDashboard.putBoolean("winch at setpoint", isWinchAtSetpoint());
    }
}
