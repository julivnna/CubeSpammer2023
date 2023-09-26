package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Reportable.LOG_LEVEL;

public class Shooter extends SubsystemBase {
    private TalonFX topMotor;
    private TalonFX bottomMotor;

    public Shooter() {
        topMotor = new TalonFX(ShooterConstants.kTopMotorID);
        bottomMotor = new TalonFX(ShooterConstants.kBottomMotorID);

        this.topMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 30, 0.1));
        this.bottomMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 25, 30, 0.1));


        topMotor.setInverted(false);
        bottomMotor.setInverted(false);

        topMotor.setNeutralMode(NeutralMode.Brake);
        bottomMotor.setNeutralMode(NeutralMode.Brake);

        topMotor.configVoltageCompSaturation(12);
        bottomMotor.configVoltageCompSaturation(12);

        topMotor.enableVoltageCompensation(true);
        bottomMotor.enableVoltageCompensation(true);

        topMotor.configNeutralDeadband(0.01);
        bottomMotor.configNeutralDeadband(0.01);
    }

    public boolean hasCube() {
        return topMotor.getSupplyCurrent() > ShooterConstants.kTopCubeCurrent.get()
            || bottomMotor.getSupplyCurrent() > ShooterConstants.kBottomCubeCurrent.get();
    }

    public CommandBase setPower(double power) {
        return runOnce(
        () -> {
            topMotor.set(ControlMode.PercentOutput, power);
            bottomMotor.set(ControlMode.PercentOutput, power);
            topMotor.setNeutralMode(NeutralMode.Brake);
            bottomMotor.setNeutralMode(NeutralMode.Brake);
        }
        );
    }

    public CommandBase setPower(double topPower, double bottomPower) {
        return runOnce(
        () -> {
            topMotor.set(ControlMode.PercentOutput, topPower);
            bottomMotor.set(ControlMode.PercentOutput, bottomPower);
            topMotor.setNeutralMode(NeutralMode.Brake);
            bottomMotor.setNeutralMode(NeutralMode.Brake);
        }
        );
    }

    public CommandBase setPowerZero() {
        return setPower(0,0);
    }

    public CommandBase outtakeFullUpdated() {
        return Commands.runOnce(() -> {
            topMotor.set(ControlMode.PercentOutput, ShooterConstants.kTopFullOuttakePower.get());
            bottomMotor.set(ControlMode.PercentOutput, ShooterConstants.kBottomFullOuttakePower.get());
        });
    }

    public CommandBase outtakeHighUpdated() {
        return Commands.runOnce(() -> {
            topMotor.set(ControlMode.PercentOutput, ShooterConstants.kTopHighOuttakePower.get());
            bottomMotor.set(ControlMode.PercentOutput, ShooterConstants.kBottomHighOuttakePower.get());
        });
    }

    public CommandBase outtakeMidUpdated() {
        return Commands.runOnce(() -> {
            topMotor.set(ControlMode.PercentOutput, ShooterConstants.kTopMidOuttakePower.get());
            bottomMotor.set(ControlMode.PercentOutput, ShooterConstants.kBottomMidOuttakePower.get());
        });
    }

    public CommandBase outtakeLowUpdated() {
        return Commands.runOnce(() -> {
            topMotor.set(ControlMode.PercentOutput, ShooterConstants.kTopLowOuttakePower.get());
            bottomMotor.set(ControlMode.PercentOutput, ShooterConstants.kBottomLowOuttakePower.get());
        });
    }


    public CommandBase intakeUpdated() {
        return Commands.runOnce(() -> {
            topMotor.set(ControlMode.PercentOutput, ShooterConstants.kTopIntakePower.get());
            bottomMotor.set(ControlMode.PercentOutput, ShooterConstants.kBottomIntakePower.get());
        });
    }

    public CommandBase holdUpdated() {
        return Commands.runOnce(() -> {
            topMotor.set(ControlMode.PercentOutput, ShooterConstants.kTopIntakeNeutralPower.get());
            bottomMotor.set(ControlMode.PercentOutput, ShooterConstants.kBottomIntakeNeutralPower.get());
        });
    }

    public CommandBase outtakeHigh() {
        return sequence(
            setPower(ShooterConstants.kTopHighOuttakePower.get(), ShooterConstants.kBottomHighOuttakePower.get()),
            waitSeconds(1),
            setPowerZero()
        );
    }

    public CommandBase outtakeMid() {
        return sequence(
            setPower(ShooterConstants.kTopMidOuttakePower.get(), ShooterConstants.kBottomMidOuttakePower.get()),
            waitSeconds(1),
            setPowerZero()
        );
    }

    public CommandBase outtakeLow() {
        return sequence(
            setPower(ShooterConstants.kTopLowOuttakePower.get(), ShooterConstants.kBottomLowOuttakePower.get()),
            waitSeconds(1),
            setPowerZero()
        );
    }

    public CommandBase intake() {
        return sequence(
            setPower(ShooterConstants.kTopIntakePower.get(), ShooterConstants.kBottomIntakePower.get()),
            waitSeconds(1),
            setPower(ShooterConstants.kTopIntakeNeutralPower.get(), ShooterConstants.kBottomIntakeNeutralPower.get())
        );
    }

    public void setNeutralMode(NeutralMode mode) {
        topMotor.setNeutralMode(mode);
        bottomMotor.setNeutralMode(mode);
    }

    public void loadPreferences() {
        ShooterConstants.kBottomIntakePower.loadPreferences();
        ShooterConstants.kBottomLowOuttakePower.loadPreferences();
        ShooterConstants.kBottomMidOuttakePower.loadPreferences();
        ShooterConstants.kBottomHighOuttakePower.loadPreferences();
        ShooterConstants.kBottomIntakeNeutralPower.loadPreferences();
        ShooterConstants.kBottomCubeCurrent.loadPreferences();
        ShooterConstants.kTopIntakePower.loadPreferences();
        ShooterConstants.kTopLowOuttakePower.loadPreferences();
        ShooterConstants.kTopMidOuttakePower.loadPreferences();
        ShooterConstants.kTopHighOuttakePower.loadPreferences();
        ShooterConstants.kTopIntakeNeutralPower.loadPreferences();
        ShooterConstants.kTopCubeCurrent.loadPreferences();
    }

    public void reportToSmartDashboard(LOG_LEVEL level) {
        switch (level) {
            case OFF:
                break;
            case ALL:
            case MEDIUM:
                SmartDashboard.putNumber("Right Shooter Velocity", bottomMotor.getSelectedSensorVelocity());
                SmartDashboard.putNumber("Left Shooter Velocity", topMotor.getSelectedSensorVelocity());
                SmartDashboard.putNumber("Right Shooter Current", bottomMotor.getStatorCurrent());
                SmartDashboard.putNumber("Left Shooter Current", topMotor.getStatorCurrent());
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
                tab.addNumber("Top Low Outtake", () -> ShooterConstants.kTopLowOuttakePower.get());
                tab.addNumber("Top Mid Outtake", () -> ShooterConstants.kTopMidOuttakePower.get());
                tab.addNumber("Top High Outtake", () -> ShooterConstants.kTopHighOuttakePower.get());
                tab.addNumber("Bottom Low Outtake", () -> ShooterConstants.kBottomLowOuttakePower.get());
                tab.addNumber("Bottom Mid Outtake", () -> ShooterConstants.kBottomMidOuttakePower.get());
                tab.addNumber("Bottom High Outtake", () -> ShooterConstants.kBottomHighOuttakePower.get());

                tab.addNumber("Right Shooter Stator Current", bottomMotor::getStatorCurrent);
                tab.addNumber("Left Shooter Stator Current", topMotor::getStatorCurrent);

                tab.addNumber("Right Shooter Supply Current", bottomMotor::getSupplyCurrent);
                tab.addNumber("Left Shooter Supply Current", topMotor::getSupplyCurrent);

                tab.add("Load Preferences", Commands.runOnce(this::loadPreferences));
            case MEDIUM:
                tab.addNumber("Right Shooter Output Voltage", bottomMotor::getMotorOutputVoltage);
                tab.addNumber("Left Shooter Output Voltage", topMotor::getMotorOutputVoltage);

                tab.addNumber("Right Shooter Output Percent", bottomMotor::getMotorOutputPercent);
                tab.addNumber("Left Shooter Output Percent", topMotor::getMotorOutputPercent);

                tab.addNumber("Right Shooter Selected Sensor Velocity", bottomMotor::getSelectedSensorVelocity);
                tab.addNumber("Left Shooter Selected Sensor Velocity", topMotor::getSelectedSensorVelocity);
            case MINIMAL:
                break;
        }

    }

    }

