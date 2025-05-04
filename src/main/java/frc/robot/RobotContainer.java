// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BiConsumer;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.Swerve_Subsystem;
import frc.robot.subsystems.wrist.Wrist_Commands;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  // public final Vision_Subsystem vision;

  public CommandXboxController driverController = new CommandXboxController(0);
  public CommandXboxController operatorController = new CommandXboxController(1);

  public Swerve_Subsystem m_swerve = new Swerve_Subsystem();
  public Wrist_Commands wrist_commands = new Wrist_Commands();

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

    final Trigger button_A = driverController.a();
    final Trigger button_B = driverController.b();
    final Trigger button_right_trigger = driverController.rightTrigger();

    BiConsumer<Trigger, Boolean> GO_TO_A = (trigger, bool) -> {
      trigger.onTrue(wrist_commands.command_go_to_A());
    };

    BiConsumer<Trigger, Boolean> GO_TO_B = (trigger, bool) -> {
      trigger.onTrue(wrist_commands.command_go_to_B());
    };

    BiConsumer<Trigger, Boolean> GRABBER_STOP = (trigger, bool) -> {
      trigger.onTrue(wrist_commands.command_grabber_stop());
    };

    GO_TO_A.accept(button_A, true);
    GO_TO_B.accept(button_B, true);
    GRABBER_STOP.accept(button_right_trigger, true);
  
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
