// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.WristConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;                     // CANSparkMax motors
import com.revrobotics.CANSparkMax.IdleMode;            // kBrake, kCost
import com.revrobotics.CANSparkMaxLowLevel.MotorType;   // kBrushless
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder;    // kDutyCycle
import com.revrobotics.SparkMaxLimitSwitch;             // Type.kNormallyOpened, Type.kNormallyClosed
// import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public final class Wrist extends SubsystemBase {
    /* Singleton Shit */
    private static Wrist m_instance = new Wrist();  // Eager instantiation

    public static Wrist getInstance() {
        return m_instance;
    }
    /* END Singleton Shit */

    private enum WristState {
        OFF,
        JOG,
        POSITION,
        ZERO
    }


    private double m_jogValue = 0.0;
    private WristState m_state = WristState.OFF;
    private Rotation2d m_setpoint = new Rotation2d();


    private final CANSparkMax m_leftMotor  = new CANSparkMax(WristConstants.kLeftMotorPort,  MotorType.kBrushless);
    private final CANSparkMax m_rightMotor = new CANSparkMax(WristConstants.kRightMotorPort, MotorType.kBrushless);

    private final AbsoluteEncoder m_encoder = m_rightMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final DigitalInput m_limitSwitch = new DigitalInput(WristConstants.kLimitSwitchPort);


    /** Creates a new Wrist. */
    private Wrist() {
        this.configMotors();
    }


    
    private void setSetpoint(final Rotation2d m_setpoint) {
        this.m_setpoint = m_setpoint;
    }

    
    public void zero() {
        this.m_jogValue = m_limitSwitch.get() ? 0.0 : 0.1;
    }




    public void configMotors() {
        m_leftMotor.follow(m_rightMotor);
        m_leftMotor.setInverted(true);

        m_leftMotor.restoreFactoryDefaults();
        m_rightMotor.restoreFactoryDefaults();

        m_leftMotor.setIdleMode(IdleMode.kBrake);
        m_leftMotor.setIdleMode(IdleMode.kBrake);

        final SparkMaxPIDController m_PIDController = m_rightMotor.getPIDController();
        m_PIDController.setFF(WristConstants.kFF);
        m_PIDController.setP(WristConstants.kP);
        m_PIDController.setD(WristConstants.kD);
    }


    @Override
    public void periodic() {
        
    }
}
