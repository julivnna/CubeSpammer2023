package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Reportable.LOG_LEVEL;

public class Shooter extends SubsystemBase {
    private TalonFX leftMotor;
    // private TalonFX rightMotor;

    public Shooter() {
        TalonFX leftMotor = new TalonFX(ShooterConstants.kLeftMotorID);
        // TalonFX rightMotor = new TalonFX(ShooterConstants.kRightMotorID);
        leftMotor.setInverted(false);
        // rightMotor.setInverted(true);
        leftMotor.setNeutralMode(NeutralMode.Brake);
        // rightMotor.setNeutralMode(NeutralMode.Brake);
        leftMotor.configVoltageCompSaturation(11);
        // rightMotor.configVoltageCompSaturation(11);
        leftMotor.enableVoltageCompensation(true);
        // rightMotor.enableVoltageCompensation(true);
    }

    public CommandBase setPower(double power) {
        return runOnce(
        () -> {
            leftMotor.set(ControlMode.PercentOutput, power);
            // rightMotor.set(ControlMode.PercentOutput, power);
            leftMotor.setNeutralMode(NeutralMode.Brake);
            // rightMotor.setNeutralMode(NeutralMode.Brake);
        }
        );
    }

    public CommandBase setPowerZero() {
        return setPower(0);
    }

    public CommandBase outtakeHigh() {
        return sequence(
            setPower(ShooterConstants.kHighOuttakePower),
            waitSeconds(0.25),
            setPowerZero()
        );
    }

    public CommandBase outtakeMid() {
        return sequence(
            setPower(ShooterConstants.kMidOuttakePower),
            waitSeconds(0.25),
            setPowerZero()
        );
    }

    public CommandBase outtakeLow() {
        return sequence(
            setPower(ShooterConstants.kLowOuttakePower),
            waitSeconds(0.25),
            setPowerZero()
        );
    }

    public CommandBase intake() {
        return sequence(
            setPower(ShooterConstants.kIntakePower),
            waitSeconds(0.25),
            setPower(ShooterConstants.kIntakeNeutralPower)
        );
    }

    public void setNeutralMode(NeutralMode mode) {
        leftMotor.setNeutralMode(mode);
        // rightMotor.setNeutralMode(mode);
    }

    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
                // SmartDashboard.putNumber("Right Shooter Velocity", rightMotor.getSelectedSensorVelocity());
                SmartDashboard.putNumber("Left Shooter Velocity", leftMotor.getSelectedSensorVelocity());
                // SmartDashboard.putNumber("Right Shooter Current", rightMotor.getStatorCurrent());
                SmartDashboard.putNumber("Left Shooter Current", leftMotor.getStatorCurrent());
            case MINIMAL:
                break;
        }
    }

    public void initShuffleboard(LOG_LEVEL level) {
        if (level == LOG_LEVEL.OFF || level == LOG_LEVEL.MINIMAL) {
            return;
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
        switch (level) {
            case OFF:
                break;
            case ALL:
                // tab.addNumber("Right Shooter Stator Current", rightMotor::getStatorCurrent);
                tab.addNumber("Left Shooter Stator Current", leftMotor::getStatorCurrent);

                // tab.addNumber("Right Shooter Supply Current", rightMotor::getSupplyCurrent);
                tab.addNumber("Left Shooter Supply Current", leftMotor::getSupplyCurrent);
            case MEDIUM:
                // tab.addNumber("Right Shooter Output Voltage", rightMotor::getMotorOutputVoltage);
                tab.addNumber("Left Shooter Output Voltage", leftMotor::getMotorOutputVoltage);

                // tab.addNumber("Right Shooter Output Percent", rightMotor::getMotorOutputPercent);
                tab.addNumber("Left Shooter Output Percent", leftMotor::getMotorOutputPercent);

                // tab.addNumber("Right Shooter Selected Sensor Velocity", rightMotor::getSelectedSensorVelocity);
                tab.addNumber("Left Shooter Selected Sensor Velocity", leftMotor::getSelectedSensorVelocity);
            case MINIMAL:
                break;
        }

    }

    }

