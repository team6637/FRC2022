// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LiftSubsystem;
import frc.robot.subsystems.PixySubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
    IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
    ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    PixySubsystem m_pixySubsystem = new PixySubsystem();
    LiftSubsystem m_liftSubsystem = new LiftSubsystem();
    Joystick stick = new Joystick(0);
    Joystick stick2 = new Joystick(1);

    public RobotContainer() {


        m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_drivetrainSubsystem,
            () -> modifyAxis(-stick.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-stick.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> modifyAxis(-stick.getTwist()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        configureButtonBindings();

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    private void configureButtonBindings() {

        // reset gyro
        new JoystickButton(stick, 11).whenPressed(m_drivetrainSubsystem::zeroGyroscope);

        // shoot
        new JoystickButton(stick, 1).whileHeld(new RunCommand(m_shooterSubsystem::shoot, m_shooterSubsystem)).whenReleased(new InstantCommand(m_shooterSubsystem::stop, m_shooterSubsystem));
                    
        // Button 2 - intake in
        new JoystickButton(stick, 2).whileHeld(new RunCommand(m_intakeSubsystem::in, m_intakeSubsystem)).whenReleased(new InstantCommand(m_intakeSubsystem::stop, m_intakeSubsystem));
        
        // Button 3  - intake and indexer out
        new JoystickButton(stick, 3).whileHeld(
             new ParallelCommandGroup(
                new RunCommand(m_intakeSubsystem::out, m_intakeSubsystem),
                new RunCommand(m_indexerSubsystem::out, m_indexerSubsystem)
             )
         ).whenReleased(
             new ParallelCommandGroup(
                new InstantCommand(m_intakeSubsystem::stop, m_intakeSubsystem),
                new InstantCommand(m_indexerSubsystem::stop, m_indexerSubsystem)
             )
         );

        // Button 4 = intake to indexer to shooter
        new JoystickButton(stick, 4).whileHeld(
            new ParallelCommandGroup(
                new RunCommand(m_indexerSubsystem::in, m_indexerSubsystem),
                new RunCommand(m_intakeSubsystem::in, m_intakeSubsystem)
            )
        ).whenReleased(
            new ParallelCommandGroup(
                new InstantCommand(m_indexerSubsystem::stop, m_indexerSubsystem),
                new InstantCommand(m_intakeSubsystem::stop, m_intakeSubsystem)
            )
        );

        // lift up
        new JoystickButton(stick2, 9).whileHeld(new RunCommand(m_liftSubsystem::up, m_liftSubsystem)).whenReleased(new InstantCommand(m_liftSubsystem::stop, m_liftSubsystem));

        // lift down
        new JoystickButton(stick2, 11).whileHeld(new RunCommand(m_liftSubsystem::down, m_liftSubsystem)).whenReleased(new InstantCommand(m_liftSubsystem::stop, m_liftSubsystem));

        // winch in
        new JoystickButton(stick2, 12).whileHeld(new RunCommand(m_liftSubsystem::in, m_liftSubsystem)).whenReleased(new InstantCommand(m_liftSubsystem::stopwinch, m_liftSubsystem));

        // winch out
        new JoystickButton(stick2, 10).whileHeld(new RunCommand(m_liftSubsystem::out, m_liftSubsystem)).whenReleased(new InstantCommand(m_liftSubsystem::stopwinch, m_liftSubsystem));

    }

    public String getAllianceColor() {
        if(stick2.getThrottle() > 0) {
          return "blue";
        } else {
          return "red";
        } 
      }



    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband - ignore really low numbers
        value = deadband(value, 0.2);

        // Square the axis for finer lower # control (.9 * .9 = .81, .2 * .2 = .4)
        value = Math.copySign(value * value, value);

        return value;
    }
}
