// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.*;
import frc.robot.Constants;
//subsystems
import frc.robot.subsystems.vision.Base_Vision_IO;
import frc.robot.subsystems.vision.Real_Limelight_Vision_IO;
import frc.robot.subsystems.vision.Vision_Subsystem;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  // public final Vision_Subsystem vision;

  public CommandXboxController driverController = new CommandXboxController(0);

  public Swerve_Subsystem m_swerve = new Swerve_Subsystem();

  public RobotContainer() {
    // Set up pathplanner
    autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    /*
     * vision = new Vision_Subsystem(m_swerve::addVisionMeasurement, // consumer
     * relies on swerve drive
     * new Real_Limelight_Vision_IO("camera_0", m_swerve::getHeading),
     * new Real_Limelight_Vision_IO("camera_1", m_swerve::getHeading)); // rotation
     * supplier relies on swerve drive
     */

    configureBindings();
  }

  private void configureBindings() {
    m_swerve.setDefaultCommand(m_swerve.controller_drive_command(driverController));

    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
