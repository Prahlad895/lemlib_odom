#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "math.h"
/*
Run 1: 




*/
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-13, -9, -14},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({15, 2, 11}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

//intake Motor 
pros::Motor intake1(1,pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
pros::Motor intake2(2,pros::v5::MotorGears::rpm_200, pros::v5::MotorUnits::degrees);
pros::Motor intake3(3,pros::v5::MotorGears::rpm_200, pros::v5::MotorUnits::degrees);

//ladybrown motor
pros::Motor LD_motor(6,pros::v5::MotorGears::green, pros::v5::MotorUnits::counts);


//Pistons
// 
//Mogo1
pros::adi::DigitalOut mogo1('B');
pros::adi::DigitalOut mogo2('C');

//Arm 1
pros::adi::DigitalOut arm1('D');
//Arm2
pros::adi::DigitalOut arm2('H');


// Inertial Sensor on port 1
pros::Imu imu(11);

// Rotation Sensor
pros::Rotation LadyDistance(10);
//Optical Sensor
pros::Optical opticalSensor(9);

// tracking wheels
/*
2.7 - 43
3.2 - 42
3 - 43
*/
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversed
pros::Rotation horizontalEnc(-15);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(12);
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
lemlib::TrackingWheel horizontal(&horizontalEnc, 2.05, -3.25);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
lemlib::TrackingWheel vertical(&verticalEnc, 2.05, -0.3125);

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral motion controller
lemlib::ControllerSettings linearController(
    3, // proportional gain (kP) 4
    0, // integral gain (kI)
    8, // derivative gain (kD)2
    5, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

// angular motion controller
lemlib::ControllerSettings angularController(3, // proportional gain (kP) 4
                                             0, // integral gain (kI)
                                             22.5, // derivative gain (kD) 26
                                             3, // anti windup
                                             1, // small error range, in inches
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in inches
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

//intake functions
void setIntake1(double power){
    intake1.move(power);
}

void setIntake2(double power){
    intake2.move(power);
}

void setIntake3(double power){
    intake3.move(power);
}
//color sort
double color =0;
double proximity =0;
bool sorting=false;
bool holding=false;





void setIntakeMotors(){
    int intakePower = 127;
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        setIntake1(intakePower);
        setIntake2(-intakePower);
        setIntake3(-intakePower);
    
    }

    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        setIntake1(intakePower);
        setIntake2(intakePower);
        setIntake3(-intakePower);
    
    }

    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        setIntake1(intakePower);
        setIntake3(intakePower);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        setIntake1(-intakePower);
        setIntake3(-intakePower);
    }
    else {
        setIntake1(0);
        setIntake2(0);
        setIntake3(0);
    }
}



bool liftTrue=true;
//ladyBrown functions
const int numStates = 2;
int states[numStates] = {0,2100};
int currState = 0;
int target = 0;

void nextState() {
    currState +=1;
/*    if (currState == 1) {
        currState = 0;
    }
    target = states[currState];*/
    target=100;
}

void nextState2() {
/*    if (currState == 1) {
        currState = 0;
    }
    target = states[currState];*/
    target=3000;
}


void liftControl(){
    while(liftTrue){
        double kp = 0.06;
        double error = target-LadyDistance.get_position();
        double velocity = (kp * error)/2;
        LD_motor.move(velocity);
    }
}

void setLady() {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            liftTrue=true;
            nextState();
        }
}

void setLady2() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
        liftTrue=true;
        nextState2();
    }
}

void setLadyManual(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        liftTrue=false;
        LD_motor.move(127);
    }
    else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        liftTrue=false;
        LD_motor.move(-127);
    }
    else{
    LD_motor.move(0);
    }
}



// mogoMech functions
bool mogoState = true; // Initialize the state variable

void setMogo() {
    // Check if the button is pressed
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        mogoState = !mogoState;  // Toggle the state
        
        // Set mogo and mogo2 based on the new state
        mogo1.set_value(mogoState);
        mogo2.set_value(mogoState);

        // Add a small delay to prevent multiple toggles from a single press
        //pros::delay(350);  // Adjust the delay as needed
    }
}
//arm functions
bool armState1 = false; // Initialize the state variable
bool armState2 = false; // Initialize the state variable

void setArm1() {
    // Check if the button is pressed
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        armState1 = !armState1;  // Toggle the state
        
        // Set arm based on the new state
        arm1.set_value(armState1);

        // Add a small delay to prevent multiple toggles from a single press
        //pros::delay(100);  // Adjust the delay as needed
    }
}

void setArm2() {
    // Check if the button is pressed
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        armState2 = !armState2;  // Toggle the state
        
        // Set arm based on the new state
        arm2.set_value(armState2);

        // Add a small delay to prevent multiple toggles from a single press
        //pros::delay(500);  // Adjust the delay as needed
    }
} 



void redNegAutonSetUp() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.setPose(-17.25,-54.5,180);
        chassis.turnToPoint(-12,-59.7,3000,{.forwards = true,.maxSpeed=90},false);
        chassis.moveToPoint(-12,-59.7,3000,{.forwards = true},false);
        chassis.turnToPoint(0,-72,3000,{.forwards = true,.maxSpeed=90},false);

    }
}

void blueNegAutonSetUp() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.setPose(-17.25,54.5,0);
        chassis.turnToPoint(-11.5,60.,3000,{.forwards = true,.maxSpeed=90},false);
        chassis.moveToPoint(-11.5,60,3000,{.forwards = true},false);
        chassis.turnToPoint(0,72,3000,{.forwards = true,.maxSpeed=90},false);
    }
}
void redPosAutonSetUp() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.setPose(30.75,-54.5,0);

        chassis.turnToPoint(34.5,-50.,3000,{.forwards = true,.maxSpeed=90},false);
        chassis.moveToPoint(34.5,-50,3000,{.forwards = true},true);

        chassis.turnToPoint(51,0.,3000,{.forwards = true,.maxSpeed=90},false);
        
       

    }
}
void bluePosAutonSetUp() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.setPose(54.25,30.5,180);
        chassis.turnToPoint(62,52.2,3000,{.forwards = false, .maxSpeed = 90}, false);

        chassis.moveToPoint(62,52.2,5000,{.forwards = false, .maxSpeed = 100}, false);
        chassis.turnToPoint(35,0.,3000,{.forwards = true,.maxSpeed=90},false);

    }
}
void redElimPosAutonSetUp() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.setPose(30.25,-54,180);
        chassis.turnToPoint(20,-20,1000,{.forwards = false},false);
    }
}
void blueElimNegAutonSetUp() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.setPose(30.25,54,0);
        chassis.turnToPoint(20,20,1000,{.forwards = false},false);
    }
}

//
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    LadyDistance.reset_position();
    LD_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	pros::Task liftControlTask([]{
		while (true) {
			liftControl();
			pros::delay(10);
		}
	});

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(1, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(2, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(3, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
  /*  pros::Task intakeControlTask([]{
		while (true) {
            opticalSensor.set_led_pwm(100);
			sort_red();
		}
	});

    pros::Task intakeHoldControlTask([]{
		while (true) {
            opticalSensor.set_led_pwm(100);
			hold_blue();
		}
	});*/
 
	//Four main functions

	//fastest way to go to a point
	chassis.moveToPoint(10,10, 1000,{.forwards = true, .maxSpeed = 127}, true);
	//move to a point with a heading "theta" is angle
	chassis.moveToPose(10,10,90, 1000,{.forwards = true, .maxSpeed = 127}, true);
	//turns toward a point
	chassis.turnToPoint(10,10,1000,{.forwards = true, .maxSpeed = 127}, true);
	//turns to angle while locking one side of the drive, provides a curve which teamsuse when alligning to mobile goals
	chassis.swingToPoint(10,10,lemlib::DriveSide::LEFT, 1000,{.forwards = true, .maxSpeed = 127}, true);

    
   
}
   

void opcontrol() {
   /* //skills
    chassis.setPose(-14.2,-58.12,132.19);
    target=19500;
    pros::delay(500);
    //Move to mogo
    chassis.moveToPoint(-20.5,-49.5,650,{.forwards = false,.minSpeed=110},false);
    mogo1.set_value(true);
    mogo2.set_value(true);
    setIntake(-127);
    chassis.turnToPoint(-25,-30,800,{.forwards = true,.maxSpeed=127,},true); 
    target=-2100;
    chassis.moveToPoint(-25,-30,700,{.forwards = true,.minSpeed=100},true);
  
    chassis.turnToPoint(-57,52,500,{.forwards = true,.maxSpeed=127},true);
    liftTrue=true;
    setIntake(0);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);*/


    
 // controller
// loop to continuously update motors

       
      
    
     while (true) {
        sorting=false;
        holding=false;
        // get joystick positions
       int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        //stickdrift correction
        if (fabs(leftY) < 10) leftY = 0;
        if (fabs(leftX) < 10) leftX = 0;
        // move the chassis with curvature drive
        chassis.arcade(leftY, leftX,false,0.5);
    

		//intake
		setIntakeMotors();
        
	/*	//ladybrown
		setLady();
        setLady2();
        setLadyManual();
		//mogoMuch
		setMogo();
		//arm
		setArm1();
        setArm2();*/
   

        
         //Test Auton Set Up
     /*    redNegAutonSetUp();
         blueNegAutonSetUp();
         redPosAutonSetUp();
         bluePosAutonSetUp();
         redElimPosAutonSetUp();
         blueElimNegAutonSetUp();
         pros::lcd::print(1, "CurrentX: %f", chassis.getPose().x); // x
         pros::lcd::print(2, "CurrentY: %f", chassis.getPose().y); // y
         pros::lcd::print(3, "CurrentTheta: %f", chassis.getPose().theta); // heading
         // log position telemetry
         lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
        
        pros::lcd::print(4, "RedNeg:(x=-14.32, y=-58.1,theta=132.48)");
        pros::lcd::print(5, "RedPos:(x=33.4,y=-51,theta=23.7)");
        pros::lcd::print(6, "BlueNeg:(x=-13.5,y=59.29,theta=47.7)");
        pros::lcd::print(7, "BluePos:(x=61.4,y=50.1,theta=202.1)");*/
        
        // delay to save resources
        pros::delay(10);
        
    }
}
