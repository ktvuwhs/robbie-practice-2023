// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivebase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public Drivebase() {}

  CANSparkMax m_leftSlave = new CANSparkMax(1, MotorType.kBrushless);
  CANSparkMax m_leftMaster = new CANSparkMax(2, MotorType.kBrushless);
  CANSparkMax m_rightSlave = new CANSparkMax(3, MotorType.kBrushless);
  CANSparkMax m_rightMaster = new CANSparkMax(4, MotorType.kBrushless);

  MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_leftSlave, m_leftMaster);
  MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_rightSlave, m_rightMaster);

  DifferentialDrive m_DifferentialDrive = new DifferentialDrive(m_leftGroup, m_rightGroup);

  m_leftSlave.follow(m_leftMaster);



  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   * 
   * 
   *
   */

  
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
