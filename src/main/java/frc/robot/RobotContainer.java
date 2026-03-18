// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Uses REVLib 2026.0.0, CTRE Phoenix v6 26.1.0, WPILib-New-Commands 1.0.0, PathplannerLib 2026.1.2

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimAndShoot;
import frc.robot.commands.EstimatePose;
import frc.robot.commands.GoAndShoot;
import frc.robot.commands.TrackCode;
import frc.robot.generated.TunerConstants_New;
import frc.robot.generated.TunerConstants_Vulcan;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WebServer;
@SuppressWarnings("unused")
public class RobotContainer {

    public static final String DRIVEBASE_TYPE = Constants.Drivebase.DRIVEBASE_TYPE;
    public final Intake intake = new Intake(24, 38, "*");
    public final Shooter shooter = new Shooter(40, 26, 48, 0, "*");
    public final CANdleSubsystem lights = new CANdleSubsystem(0, "*");
    public final XboxController joystick0 = new XboxController(0);
    public final XboxController joystick1 = new XboxController(1);
    
    public static double SpeedReduction = Constants.Drivebase.SpeedReduction;
    private final double DEADBAND_VALUE = Constants.Drivebase.DEADBAND_VALUE;

    // NOT CHANGEABLE CONSTANTS
    public static double speedChange = 1;
    private double MaxSpeed = TunerConstants_Vulcan.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final Telemetry logger = new Telemetry(MaxSpeed);
    
    public final CommandSwerveDrivetrain drivetrain = createDrivetrain();
    private final WebServer webServer = new WebServer(drivetrain, shooter);

    private static CommandSwerveDrivetrain createDrivetrain() {
    switch (DRIVEBASE_TYPE) {
        case "2026":
            return TunerConstants_New.createDrivetrain();
        case "vulcan":
            return TunerConstants_Vulcan.createDrivetrain();
        default:
            throw new IllegalArgumentException("Unknown drivebase: " + DRIVEBASE_TYPE);
        }
    }

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        NamedCommands.registerCommand("AutoAlignAndShoot",new AimAndShoot(drivetrain,shooter,lights,() -> true,joystick0));

        NamedCommands.registerCommand("ZeroRobotBase",drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        NamedCommands.registerCommand("LimelightSync",new EstimatePose(lights,drivetrain,false));

        autoChooser = AutoBuilder.buildAutoChooser("Taxi");SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        
    }

    private void configureBindings() {

        // Joystick0 - Main Driver Controller
        
        new Trigger(() -> joystick0.getStartButton()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        new Trigger(() -> joystick0.getPOV() == 90).onTrue(new InstantCommand(() -> speedChange = 0.3)).onFalse(new InstantCommand(() -> speedChange = 1.0));
        
        new Trigger(() -> joystick0.getPOV() == 180).whileTrue(new TrackCode(drivetrain, joystick0, lights, true));
        
        new Trigger(() -> joystick0.getPOV() == 270).whileTrue(new GoAndShoot(drivetrain, shooter, lights));
        
        new Trigger(() -> joystick0.getPOV() == 360).whileTrue(new EstimatePose(lights,drivetrain,true));
        
        new Trigger(() -> joystick0.getLeftBumper()).whileTrue(new AimAndShoot(drivetrain,shooter,lights,() -> true,joystick0));
        
        new Trigger(() -> joystick0.getRightBumper()).whileTrue(intake.run(() -> intake.timedArmDown())).onFalse(intake.runOnce(intake::stop));
        
        new Trigger(() -> joystick0.getLeftTriggerAxis() > 0.2).whileTrue(intake.run(() -> intake.setIntake(joystick0.getLeftTriggerAxis()))).onFalse(intake.runOnce(intake::stop));
        
        new Trigger(() -> joystick0.getRightTriggerAxis() > 0.05).whileTrue(shooter.run(() -> shooter.runRotationsAsController(joystick0.getRightTriggerAxis()))).onFalse(shooter.runOnce(shooter::stop));
                            
        // Joystick1 - Operator Controller

        new Trigger(() -> joystick1.getStartButton()).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        new Trigger(() -> joystick1.getPOV() == 90).onTrue(new InstantCommand(() -> speedChange = 0.3)).onFalse(new InstantCommand(() -> speedChange = 1.0));
        
        new Trigger(() -> joystick1.getPOV() == 180).whileTrue(new TrackCode(drivetrain, joystick1, lights, true));
        
        new Trigger(() -> joystick1.getPOV() == 270).whileTrue(new GoAndShoot(drivetrain, shooter, lights));
        
        new Trigger(() -> joystick1.getPOV() == 360).whileTrue(new EstimatePose(lights,drivetrain,true));
        
        new Trigger(() -> joystick1.getLeftBumper()).whileTrue(new AimAndShoot(drivetrain,shooter,lights,() -> true,joystick1));
        
        new Trigger(() -> joystick1.getRightBumper()).whileTrue(intake.run(() -> intake.timedArmDown())).onFalse(intake.runOnce(intake::stop));
        
        new Trigger(() -> joystick1.getLeftTriggerAxis() > 0.2).whileTrue(intake.run(() -> intake.setIntake(joystick1.getLeftTriggerAxis()))).onFalse(intake.runOnce(intake::stop));
        
        new Trigger(() -> joystick1.getRightTriggerAxis() > 0.05).whileTrue(shooter.run(() -> shooter.runRotationsAsController(joystick1.getRightTriggerAxis()))).onFalse(shooter.runOnce(shooter::stop));

        // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-MathUtil.applyDeadband(joystick0.getLeftY(), DEADBAND_VALUE) * MaxSpeed * SpeedReduction * speedChange)
        //             .withVelocityY(-MathUtil.applyDeadband(joystick0.getLeftX(), DEADBAND_VALUE) * MaxSpeed * SpeedReduction * speedChange)
        //             .withRotationalRate(-MathUtil.applyDeadband(joystick0.getRightX(), DEADBAND_VALUE) * MaxAngularRate * SpeedReduction * speedChange)
        //             )
        //         );

        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                boolean joystick0Active = Math.abs(joystick0.getLeftY()) > DEADBAND_VALUE || Math.abs(joystick0.getLeftX()) > DEADBAND_VALUE || Math.abs(joystick0.getRightX()) > DEADBAND_VALUE;
                var active = joystick0Active ? joystick0 : joystick1;
                return drive.withVelocityX(-MathUtil.applyDeadband(active.getLeftY(), DEADBAND_VALUE) * MaxSpeed * SpeedReduction * speedChange)
                            .withVelocityY(-MathUtil.applyDeadband(active.getLeftX(), DEADBAND_VALUE) * MaxSpeed * SpeedReduction * speedChange)
                            .withRotationalRate(-MathUtil.applyDeadband(active.getRightX(), DEADBAND_VALUE) * MaxAngularRate * SpeedReduction * speedChange);
            })
        );

        // Idle & Telemetry
                    
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
    String selected = WebServer.getSelectedAuto();

    Command zeroCommand = new InstantCommand(() -> {
        drivetrain.seedFieldCentric();
        System.out.println("Zeroed Robot.");
    });

    Command autoCommand;

    if (selected == null || selected.isEmpty()) {
        System.out.println("No auto selected, running Taxi.");
        autoCommand = drivetrain.run(
            () -> drivetrain.applyRequest(
                () -> new SwerveRequest.RobotCentric()
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                    .withVelocityX(0.3)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
        ).withTimeout(2.0);
    } else {
        autoCommand = AutoBuilder.buildAuto(selected);
    }

    return new SequentialCommandGroup(
        zeroCommand,
        autoCommand
        );
    }
}