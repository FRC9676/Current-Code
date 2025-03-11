// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//all of the imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
  private Joystick m_Driver; //driver controller
  private Joystick m_Elevator; //Elevator controller
  //motor ID #s
  private static final int rightFrontDriveeID = 1;
  private static final int leftRearDriveID = 2;
  private static final int rightRearDriveID = 3;
  private static final int leftFrontDriveID = 4;
  private static final int ElevatorID = 5;
  private static final int intakeBottomID = 8;
  private static final int intakeTopID = 7;
  private static final int climberID = 10;
  //kraken motors
  private final TalonFX m_leftFront = new TalonFX(leftFrontDeviceID);
  private final TalonFX m_leftRear = new TalonFX(leftRearDeviceID);
  private final TalonFX m_rightFront = new TalonFX(rightFrontDeviceID);
  private final TalonFX m_rightRear = new TalonFX(rightRearDeviceID);
  private final DutyCycleOut leftOut = new DutyCycleOut(0);
  private final DutyCycleOut rightOut = new DutyCycleOut(0);
  private final CurrentLimitsConfigs m_currentLim = new CurrentLimitsConfigs();
  //neo motors
  private CANSparkMax m_Elevator;
  private CANSparkMax m_intakeBottom;
  private CANSparkMax m_intakeTop;
  private CANSparkMax m_intake;
  private CANSparkMax m_climber;
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
    m_Elevator = new CANSparkMax(ElevatorID, MotorType.kBrushless);
    m_intakeBottom = new CANSparkMax(intakeBottomID, MotorType.kBrushless);
    m_intakeTop = new CANSparkMax(intakeTopID, MotorType.kBrushless);
    m_intake = new CANSparkMax(intake, MotorType.kBrushless);
    m_climber = new CANSparkMax(climberID, MotorType.kBrushless);

    //robot setup
    m_myRobot = new DifferentialDrive(m_leftFront, m_rightFront);

    //driver channel stuff
    m_Driver = new Joystick(0);
    m_Elevator = new Joystick(1);
    m_Driver.setXChannel(4);
    m_Driver.setYChannel(1);
    m_Elevator.setXChannel(5);

    //kraken limits
    m_currentLim.SupplyCurrentLimit = 1;
    m_currentLim.SupplyCurrentThreshold = 4;
    m_currentLim.SupplyTimeThreshold = 1.0;
    m_currentLim.StatorCurrentLimitEnable = true;
  }

   //motor vars
   static boolean intakeOn = false;
   static boolean intakeMode = false;
   static int climber = 2; //if the climber is moving, 0 = up, 1 = down, 2 = home 
   static double climberHome; //sets the home position of the climber
   private RelativeEncoder climberEncoder; //the climber encoder
   static double fwd;//drivetrain forward var
   static double rot;//drivetrain rotation var

   //Elevator motor controller
   public void setElevator(double percent){
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
  public void setDist(int right, int left){
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
    //timeRun = Timer.getFPGATimestamp() - autoTimeStart;

    //m_autoSelected = m_chooser.getSelected();

    //leftPos = m_leftFront.getPosition().getValue();
    //rightPos = m_rightFront.getPosition().getValue();

    //if(m_autoSelected == "Center"){
      //if(step == 0){//arm to shoot pos and spin up motors
        if(encoder.getAbsolutePosition() < 0.9){
          setArmMotor(-0.3);
        }else if(encoder.getAbsolutePosition() < 0.95){
          setArmMotor(-0.1);
          shootOn = true;
        }else if(encoder.getAbsolutePosition() >= 0.94){
          if(encoder.getAbsolutePosition() >= 0.96){
            setArmMotor(0.1);
            shootOn = true;
            step = 1;
        }else{
          setArmMotor(0.0);
        }
        }else{
          setArmMotor(0);
        }
      }else if(step == 1){//shoot note
          intakeMode = true;
          intakeOn = true;
          setArmMotor(0);
        if(timeRun > 4.5){
          intakeOn = false;
          intakeMode = false;
          shootOn = false;
          if(encoder.getAbsolutePosition() < 0.96){
            setArmMotor(-0.2);
          }else if(encoder.getAbsolutePosition() < 0.985){
            setArmMotor(-0.15);
          }else if(encoder.getAbsolutePosition() >= 0.985){
            setArmMotor(0);
            nextStep();
          }  
        }
      }else if(step == 2){//move to first note and pick it up
        setDist(40,40);
      }else if(step == 3){//return home
        shootOn = true;
        home();
      }else if(step == 4){//raise arm
        if(encoder.getAbsolutePosition() < 0.9){
          setArmMotor(-0.2);
          shootOn = true;
        }else if(encoder.getAbsolutePosition() < 0.95){
          setArmMotor(-0.1);
          shootOn = true;
        }else if(encoder.getAbsolutePosition() >= 0.94){
          if(encoder.getAbsolutePosition() >= 0.96){
            setArmMotor(0.1);
        }else{
          setArmMotor(0.05);
          nextStep();
        }
        }else{
          setArmMotor(0);
        }
      }else if(step == 5){//shoot
        if(timeRun < 12){
          setArmMotor(0);
          intakeMode = true;
          intakeOn = true;
        }else if(timeRun >= 12){
          setArmMotor(0);
          intakeOn = false;
          intakeMode = false;
          shootOn = false;
          if(timeRun > 13){
            nextStep();
          }
        }
      }else if(step == 6){
        setDist(40,40);
        intakeOn = false;
        intakeMode = false;
      }
    
    
    if(m_autoSelected == "Red Amp Side"){
      if(step == 0){//arm to shoot pos and spin up motors
        if(encoder.getAbsolutePosition() < 0.9){
          setArmMotor(-0.3);
        }else if(encoder.getAbsolutePosition() < 0.95){
          setArmMotor(-0.1);
          shootOn = true;
        }else if(encoder.getAbsolutePosition() >= 0.94){
          if(encoder.getAbsolutePosition() >= 0.96){
            setArmMotor(0.1);
            shootOn = true;
            step = 1;
        }else{
          setArmMotor(0.0);
        }
        }else{
          setArmMotor(0);
        }
      }else if(step == 1){//shoot note
          intakeMode = true;
          intakeOn = true;
          setArmMotor(0);
        if(timeRun > 4.5){
          intakeOn = false;
          intakeMode = false;
          shootOn = false;
          if(encoder.getAbsolutePosition() < 0.96){
            setArmMotor(-0.2);
          }else if(encoder.getAbsolutePosition() < 0.985){
            setArmMotor(-0.15);
          }else if(encoder.getAbsolutePosition() >= 0.985){
            setArmMotor(0);
            nextStep();
          }  
        }
      }else if(step == 2){//move to first note and pick it up
        setDist(88, 20);
      }else if(step == 3){//return home
        shootOn = true;
        home();
      }else if(step == 4){//raise arm
        if(encoder.getAbsolutePosition() < 0.9){
          setArmMotor(-0.2);
          shootOn = true;
        }else if(encoder.getAbsolutePosition() < 0.95){
          setArmMotor(-0.1);
          shootOn = true;
        }else if(encoder.getAbsolutePosition() >= 0.94){
          if(encoder.getAbsolutePosition() >= 0.96){
            setArmMotor(0.1);
        }else{
          setArmMotor(0.05);
          nextStep();
        }
        }else{
          setArmMotor(0);
        }
      }else if(step == 5){//shoot
        if(timeRun < 12){
          setArmMotor(0);
          intakeMode = true;
          intakeOn = true;
        }else if(timeRun >= 12){
          setArmMotor(0);
          intakeOn = false;
          intakeMode = false;
          shootOn = false;
          if(timeRun > 13){
            nextStep();
          }
        }
      }else if(step == 6){//
        intakeOn = false;
        intakeMode = false;
      }
    }

    if(m_autoSelected == "Blue Amp Side"){
      if(step == 0){//arm to shoot pos and spin up motors
        if(encoder.getAbsolutePosition() < 0.9){
          setArmMotor(-0.3);
        }else if(encoder.getAbsolutePosition() < 0.95){
          setArmMotor(-0.1);
          shootOn = true;
        }else if(encoder.getAbsolutePosition() >= 0.94){
          if(encoder.getAbsolutePosition() >= 0.96){
            setArmMotor(0.1);
            shootOn = true;
            step = 1;
        }else{
          setArmMotor(0.0);
        }
        }else{
          setArmMotor(0);
        }
      }else if(step == 1){//shoot note
          intakeMode = true;
          intakeOn = true;
          setArmMotor(0);
        if(timeRun > 4.5){
          intakeOn = false;
          intakeMode = false;
          shootOn = false;
          if(encoder.getAbsolutePosition() < 0.96){
            setArmMotor(-0.2);
          }else if(encoder.getAbsolutePosition() < 0.985){
            setArmMotor(-0.15);
          }else if(encoder.getAbsolutePosition() >= 0.985){
            setArmMotor(0);
            nextStep();
          }  
        }
      }else if(step == 2){//move to first note and pick it up
        setDist(20, 88);
      }else if(step == 3){//return home
        shootOn = true;
        home();
      }else if(step == 4){//raise arm
        if(encoder.getAbsolutePosition() < 0.9){
          setArmMotor(-0.2);
          shootOn = true;
        }else if(encoder.getAbsolutePosition() < 0.95){
          setArmMotor(-0.1);
          shootOn = true;
        }else if(encoder.getAbsolutePosition() >= 0.94){
          if(encoder.getAbsolutePosition() >= 0.96){
            setArmMotor(0.1);
        }else{
          setArmMotor(0.05);
          nextStep();
        }
        }else{
          setArmMotor(0);
        }
      }else if(step == 5){//shoot
        if(timeRun < 12){
          setArmMotor(0);
          intakeMode = true;
          intakeOn = true;
        }else if(timeRun >= 12){
          setArmMotor(0);
          intakeOn = false;
          intakeMode = false;
          shootOn = false;
          if(timeRun > 13){
            nextStep();
          }
        }
      }else if(step == 6){//
        intakeOn = false;
        intakeMode = false;
      }
    }
  }

  double climberPos;

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    encoder.reset();
    climberEncoder = m_climber.getEncoder();
    climberHome = climberEncoder.getPosition();
  }

  /** This function is called periodically during operator control. */

  @Override
  public void teleopPeriodic() 
    //SmartDashboard.putNumber("Pivot Read Out", encoder.getAbsolutePosition()); //read out encoder pos
    //SmartDashboard.putNumber("Climb", climberEncoder.getPosition());
    //SmartDashboard.putBoolean("Laser", laser.get());
    //SmartDashboard.putBoolean("Laser2", laser2.get());
    //SmartDashboard.putNumber("Shooter", shooterSpeed);
    //climberPos = climberEncoder.getPosition();
    //kraken stuff
    //leftOut.Output = fwd + rot;
    //rightOut.Output = fwd - rot;
    //m_leftFront.setControl(leftOut);
    //m_rightFront.setControl(rightOut);
    //drive mode
    //if(m_Driver.getRawButton(6)){//nitrous active
      //fwd = -m_Driver.getY()*.35;
      //rot = m_Driver.getX()*0.3;
    //}else{//regular mode
      //fwd = -m_Driver.getY()*0.25;
      //rot = m_Driver.getX()*0.2;
    //}
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
    //climbing control
    //if(m_Arm.getX() > 0.06 ){
      //climber = 0;
    //}else if(m_Arm.getX() < -0.06){
      //climber = 1;
    //}else{
     // climber = 2;
    //}
    //if(climber == 0){
      //if(climberEncoder.getPosition() > 152){
        //m_climber.set(0);
      //}else{
        //m_climber.set(1);
      //}
    //}else if(climber == 1){
      //if(climberEncoder.getPosition() < -85){
        //m_climber.set(0);
      //}else{
        //m_climber.set(-1);
      //}
    //}else{
      //m_climber.set(0);
    //}
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
