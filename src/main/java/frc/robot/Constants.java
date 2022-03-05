// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants {
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(23.5);
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(23.5);
    public static final int DRIVETRAIN_PIGEON_ID = 30;

    // least amount of power required to overcome the drivetrain's static friction
    public static final double DRIVETRAIN_KS = 0.2;
    
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 21; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(346.46); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 22; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(221.84);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 23; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(347.69);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 24; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(144.58); 

    // CAN IDs
    public static final int SHOOTER_LEFT = 52;
    public static final int SHOOTER_RIGHT = 51;
    public static final int INDEXER = 53;
    public static final int INTAKE = 54;
    public static final int LIFT_LEFT = 55;
    public static final int LIFT_RIGHT = 56;
    public static final int WINCH = 57;
    public static final int HOOD = 58;

    // DIO Ports
    public static final int SHOOTER_ENCODER_1 = 0;
    public static final int SHOOTER_ENCODER_2 = 1;
    public static final int LIFT_LEFT_ENCODER = 2;
    public static final int LIFT_RIGHT_ENCODER = 4;
    public static final int LIFT_WINCH_ENCODER = 3;
    public static final int INTAKE_BALL_SENSOR_1 = 6;
    public static final int INTAKE_BALL_SENSOR_2 = 7;


}
