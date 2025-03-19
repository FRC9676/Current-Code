// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


//all of the imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Center";
  private static final String kCustomAuto = "Red Amp Side";
  private static final String kCustomAuto2 = "Blue Amp Side";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  //controllers
  private DifferentialDrive m_myRobot;
  private Joystick j_Driver; //driver controller
  private Joystick j_Elevator; //Elevator controller
  //motor ID #s
  private static final int rightFrontDriveeID = 1;
  private static final int leftRearDriveID = 2;
  private static final int rightRearDriveID = 3;
  private static final int leftFrontDriveID = 4;
  private static final int ElevatorID = 5;
  private static final int intakeBottomID = 6;
  private static final int intakeTopID = 7;
  //private static final int climberID = 10;
  //kraken motors
  private final TalonFX m_leftFront = new TalonFX(leftFrontDriveID);
  private final TalonFX m_leftRear = new TalonFX(leftRearDriveID);
  private final TalonFX m_rightFront = new TalonFX(rightFrontDriveeID);
  private final TalonFX m_rightRear = new TalonFX(rightRearDriveID);
  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);
  private final CurrentLimitsConfigs m_currentLim = new CurrentLimitsConfigs();
  //neo motors
  private SparkMax m_Elevator; 
  private SparkMax m_intakeBottom;
  private SparkMax m_intakeTop;
  private SparkMax m_intake;
  //private SparkMax m_climber;

  static final DutyCycleEncoder encoder = new DutyCycleEncoder(0); //pivot encoder
  DigitalInput laser = new DigitalInput(4);
  static final double kP = 0;
  static final double kI = 0;
  static final double kD = 0;
  PIDController pid = new PIDController(kP, kI, kD);
  DigitalInput laser2 = new DigitalInput(6);

  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Center", kDefaultAuto);
    m_chooser.addOption("Red Amp Side", kCustomAuto2);
    m_chooser.addOption("Blue Amp Side", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //kraken setup
    var leftConfiguration = new TalonFXConfiguration();
    var rightConfiguration = new TalonFXConfiguration();
    leftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    m_leftFront.getConfigurator().apply(leftConfiguration);
    m_leftRear.getConfigurator().apply(leftConfiguration);
    m_rightFront.getConfigurator().apply(rightConfiguration);
    m_rightRear.getConfigurator().apply(rightConfiguration);
    m_leftRear.setControl(new Follower(m_leftFront.getDeviceID(), false));
    m_rightRear.setControl(new Follower(m_rightFront.getDeviceID(), false));
    m_leftFront.setSafetyEnabled(true);
    m_rightFront.setSafetyEnabled(true);
    
    //Elevator setup
    m_Elevator = new SparkMax(ElevatorID, MotorType.kBrushless);
    m_intakeBottom = new SparkMax(intakeBottomID, MotorType.kBrushless);
    m_intakeTop = new SparkMax(intakeTopID, MotorType.kBrushless);
    m_intake = new SparkMax(intake, MotorType.kBrushless);
    //m_climber = new SparkMax(climberID, MotorType.kBrushless);

    //robot setup
    m_myRobot = new DifferentialDrive(m_leftFront::set, m_rightFront::set);

    //driver channel stuff
    j_Driver = new Joystick(0);
    j_Elevator = new Joystick(1);
    j_Driver.setXChannel(2);
    j_Driver.setYChannel(1);
    j_Elevator.setXChannel(5);

    //kraken limits
    m_currentLim.SupplyCurrentLimit = 1;
    m_currentLim.SupplyCurrentThreshold = 4;
    m_currentLim.SupplyTimeThreshold = 1.0;
    m_currentLim.StatorCurrentLimitEnable = true;
  }

   //motor vars
   static boolean intakeOn = false;
   static boolean intakeMode = false;
   static int elevator_preset = 0; //0 = home, 1 = coral tray, 2 = reef lvl1, 3 = reef lvl2, 4 = reef lvl3  
   static double elevatorHome; //sets the home position of the elevator
   private RelativeEncoder elevatorEncoder; //the elevator encoder
   static double fwd;//drivetrain forward var
   static double rot;//drivetrain rotation var

    Intake motor controller
    public void setIntake(double percent){
    m_Elevator.set(percent);

    if(intakeOn == true){//intake setting
      if(intakeMode == false){ //sucky uppy
        m_intake.set(0.2);
      }else if(intakeMode == true){ //shoot mode
        m_intake.set(1);
      }
    }else{
      m_intake.set(0);
    }
  } 

  @Override
  public void robotPeriodic() {}

  //auton vars
  double autoTimeStart; //the time auton is started
  double timeRun; //the time auton has run
  double stepTime; //the time the last step stopped

  double posTargetLeft;
  double posTargetRight;
  double leftPos;
  double rightPos;
  double var;
  double leftSpeed; //sets the left speed
  double rightSpeed; //sets the right speed
  double lastLeft;
  double lastRight;
  int step;

  //8.46 gear ratio
  //18.84 inches per revolution
    public void setDist(int right, int left) {
    posTargetRight = ((right/18.84)*8.46) + lastRight;
    posTargetLeft = ((left/18.84)*8.46) + lastLeft;

    if(left >= right){
      var = left*8;
      leftSpeed = (left/var);
      rightSpeed = (right/var);
    }
    if(left < right){
      var = right*8;
      leftSpeed = -(left/var);
      rightSpeed = -(right/var);
    }
    if(rightPos >= posTargetRight){
      rightSpeed = 0;
    }else{
      m_rightFront.set(rightSpeed);
    }
    if(leftPos >= posTargetLeft){
      leftSpeed = 0;
    }else{
      m_leftFront.set(leftSpeed);
    }

    if(leftSpeed == 0){
      if(rightSpeed == 0){
        step = step + 1;
        lastRight = m_rightFront.getPosition().getValue();
        lastLeft = m_leftFront.getPosition().getValue();
        stepTime = Timer.getFPGATimestamp() - stepTime;
      }
    }
  }

  public void home(){
    posTargetRight = 0;
    posTargetLeft = 0;

    if(leftPos >= rightPos){
      var = leftPos*8;
      leftSpeed = -(leftPos/var);
      rightSpeed = -(rightPos/var);
    }
    if(leftPos < rightPos){
      var = rightPos*8;
      leftSpeed = -(leftPos/var);
      rightSpeed = -(rightPos/var);
    }

    if(rightPos <= posTargetRight){
      rightSpeed = 0;
    }
    if(leftPos <= posTargetLeft){
      leftSpeed = 0;
    }

    m_rightFront.set(rightSpeed);
    m_leftFront.set(leftSpeed);


    if(leftSpeed == 0){
      if(rightSpeed == 0){
        step = step + 1;
        lastRight = m_rightFront.getPosition().getValue();
        lastLeft = m_leftFront.getPosition().getValue();
        stepTime = Timer.getFPGATimestamp() - stepTime;
      }
    }
  }

  public void nextStep(){
    step = step + 1;
    stepTime = Timer.getFPGATimestamp() - stepTime;
  }
 
   @Override
  public void autonomousInit() {
    System.out.println("Auto selected: " + m_autoSelected);
    autoTimeStart = Timer.getFPGATimestamp();

    m_leftFront.setPosition(0);
    m_rightFront.setPosition(0);

    step = 0;
    lastLeft = 0;
    lastRight = 0;
  } 
//EVERYTHING BELOW THIS IS OVERRIDE
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }
   
  double elevatorPos;

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //encoder.reset();
    elevatorEncoder = m_Elevator.getEncoder();
    elevatorHome = elevatorEncoder.getPosition();
  }

  /** This function is called periodically during operator control. */

  @Override
  public void teleopPeriodic() { 
    SmartDashboard.putNumber("Pivot Read Out", encoder.get()); //read out encoder pos
    SmartDashboard.putNumber("Elevator", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Y", -j_Driver.getY());
    SmartDashboard.putNumber("X", -j_Driver.getX());
    SmartDashboard.putBoolean("Laser", laser.get());
    SmartDashboard.putBoolean("Laser2", laser2.get());
    //SmartDashboard.putNumber("Shooter", shooterSpeed);
    elevatorPos = elevatorEncoder.getPosition();
    //kraken stuff
    m_myRobot.arcadeDrive(-j_Driver.getY()*.25, -j_Driver.getX()*.25);

    /* //leftOut.Output = fwd + rot;
    rightOut.Output = fwd - rot;
    m_leftFront.setControl(leftOut);
    m_rightFront.setControl(rightOut);
    //drive mode
    if(j_Driver.getRawButton(6)){//nitrous active
      fwd = -j_Driver.getY()*.35;
      rot = j_Driver.getX()*0.3;
    }else{//regular mode
      fwd = -j_Driver.getY()*0.25;
      rot = j_Driver.getX()*0.2;
    } */
    //intake note
    //if(m_Arm.getRawButton(1)){
      //if(laser.get() == laser2.get()){
        //if(laser.get() == false){
          //intakeOn = false;
        //}else{
          //intakeOn = true;
        //}
      //}else{
        //intakeOn = true;
      //}
    //}else{
      //intakeOn = false;
    //}
    //shoot spinup
    //if(m_Arm.getRawButton(5)){
      //shootOn = true;
    //}else{
      //shootOn = false;
    //}

    //if(m_Arm.getRawButton(3)){ //e-shoot
      //shooterSpeed = 0.7;
    //}else{
      //shooterSpeed = 0.5;
    //}
    //shoot the note
    //if(m_Arm.getRawButton(6)){
      //if(shootOn == true){
        //intakeMode = true;
        //intakeOn = true;
      //}  
    //}else{
      //intakeMode = false;
    //}
    //elevator control
    //m_Elevator.set(0);
    m_Elevator.set(-j_Elevator.getY()*.25);

    //button mappings based on https://github.com/mandlifrc/GearsBotF310
    //HOPE THEY WORK!!!!!!!!!!
     if(j_Elevator.getRawButton(3)){
      elevator_preset = 1;
    }else if(j_Elevator.getRawButton(1)){
      elevator_preset = 2;
    }else if(j_Elevator.getRawButton(2)){
      elevator_preset = 3;
    }else if(j_Elevator.getRawButton(4)){
      elevator_preset = 4;
    }else{
      elevator_preset = 0;
    }
    if(elevator_preset == 0){
      if(elevatorEncoder.getPosition() > -30){
        m_Elevator.set(-1);
      }else{
        m_Elevator.set(0);
      }
    }else if(elevator_preset == 1){
      if(elevatorEncoder.getPosition() < -15){
        m_Elevator.set(1);
      }else if(elevatorEncoder.getPosition() > -15){
        m_Elevator.set(-1);
      }else{
        m_Elevator.set(0);
      }
    }else if(elevator_preset == 2){
      if(elevatorEncoder.getPosition() < 4){
        m_Elevator.set(1);
      }else if(elevatorEncoder.getPosition() > 4){
        m_Elevator.set(-1);
      }else{
        m_Elevator.set(0);
      }
    }else if(elevator_preset == 3){
      if(elevatorEncoder.getPosition() < 23){
        m_Elevator.set(1);
      }else if(elevatorEncoder.getPosition() > 23){
        m_Elevator.set(-1);
      }else{
        m_Elevator.set(0);
      }
    }else if(elevator_preset == 4){
      if(elevatorEncoder.getPosition() < 42){
        m_Elevator.set(1);
      }else if(elevatorEncoder.getPosition() > 42){
        m_Elevator.set(-1);
      }else{
        m_Elevator.set(0);
      }
    }else{
      m_Elevator.set(0);
    } 
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
