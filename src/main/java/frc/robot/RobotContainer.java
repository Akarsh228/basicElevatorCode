// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  CommandXboxController driver = new CommandXboxController(0);
  elevator elevator = new elevator();

  public RobotContainer() {
    
    configureBindings();
  }

  private void configureBindings() {
    driver.a().onTrue(elevator.setElevatorPosition(1000)); 

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
