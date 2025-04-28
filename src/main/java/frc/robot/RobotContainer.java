// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.Constants;
//subsystems
import frc.robot.subsystems.vision.Vision_IO_Base;
import frc.robot.subsystems.vision.Vision_IO_Real_Limelight;
import frc.robot.subsystems.vision.Vision_Subsystem;


public class RobotContainer {

  Vision_Subsystem vision;

  public RobotContainer() {

    switch (Constants.current_mode) {
      case REAL:
        vision = new Vision_Subsystem(null, // consumer relies on swerve drive
            new Vision_IO_Real_Limelight("camera_0", null),
            new Vision_IO_Real_Limelight("camera_1", null)); // rotation supplier relies on swerve drive

        break;
    
      case SIM:
        
        break;

      case REPLAY:

        break;

      default:
      
        break;
    }
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
