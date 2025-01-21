package frc.robot;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class elevator extends SubsystemBase {

  public TalonFX motor;

  public Elevator() {
    motor = new TalonFX(45);

    TalonFXConfiguration elevatorMMConfig = new TalonFXConfiguration();
    Slot0Configs elevatorConfigs = new Slot0Configs();

    elevatorConfigs.kP = 5;
    elevatorConfigs.kI = 0;
    elevatorConfigs.kD = 0;

    elevatorConfigs.GravityType = GravityTypeValue.Elevator_Static;
    elevatorConfigs.kG = 1;

    elevatorMMConfig.CurrentLimits.SupplyCurrentLimit = 80;
    elevatorMMConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevatorMMConfig.CurrentLimits.StatorCurrentLimit = 80;
    elevatorMMConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    elevatorMMConfig.Feedback.FeedbackRemoteSensorID = motor.getDeviceID();
    elevatorMMConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    elevatorMMConfig.MotionMagic.MotionMagicAcceleration = 1;
    elevatorMMConfig.MotionMagic.MotionMagicCruiseVelocity = 0.7;
    elevatorMMConfig.MotionMagic.MotionMagicJerk = 0.1;
    elevatorMMConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorMMConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorMMConfig.withSlot0(elevatorConfigs);

    motor.getConfigurator().apply(elevatorMMConfig);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("elevator motor pos", motor.getPosition().getValueAsDouble());
    SmartDashboard.updateValues();
  }

  public Command setElevatorPosition(double pos) {
    return runOnce(() -> {
        motor.setControl(new MotionMagicExpoVoltage(pos));
      }
    );
  }


}