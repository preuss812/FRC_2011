#include "WPILib.h"
#include "math.h"

/*
 * Drive Motor 1 (FL) - Jaguar - PWM 1
 * Drive Motor 2 (FR) - Jaguar - PWM 2
 * Drive Motor 3 (RL) - Jaguar - PWM 3
 * Drive Motor 4 (RR) - Jaguar - PWM 4

 * Lift Motor - Jaguar - PWM 5
 * Arm Rotation - 2 Jaguar - PWM 6 split

 * Arm Pneumatics - 2 Relays - 
 * Minibot Deployment - Spike - 
 * Compressor - Spike - 
 * Compressor PSI switch - Digitial IO X
 */

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */
class RobotDemo : public SimpleRobot //DECLARING
{
	RobotDrive *myRobot; // robot drive system
	Joystick *leftstick; // only joystick
	Joystick *rightstick; // only joystick
	Joystick *armstick;
	Relay *k_relay; // not the only relay
	Relay *led_relay; // LED lights for autonomous
	Compressor *compressor; // Compressor
	Solenoid *soy[4]; //sauce  
	Jaguar *forkliftjag; //jag for forklift
	Jaguar *flexjag;//jag for flexing the grabber arm hand

	int piston_position; // 0 down, 1 up TOOOO DDDOOOOO!!!!!! CHANGE TO GRAB AND RELEASE
	int grab_position; // 0 means up and 1 means down
	int minibot_deployed;
	Task *cameraTask;
	Task *teleCameraTask;
	//Task *flexTask;
	//int flex_direction;

	
public:
	RobotDemo(void) //CREATING
	{
		myRobot = new RobotDrive(1, 3, 2, 4);// these must be initialized in the same order FL, RL, FR, RR
		leftstick = new Joystick(1); // as they are declared above.
		rightstick = new Joystick(2); // as they are declared above.
		armstick = new Joystick(3);
		k_relay = new Relay(2,Relay::kForwardOnly);
		led_relay = new Relay(3,Relay::kForwardOnly);
		compressor = new Compressor(1,1);//Digital IO 1, Relay 1
		soy[0]= new Solenoid(1);//solenoid to open to close the jaws
		soy[1]= new Solenoid(2);//solenoid to open to open the jaws
		soy[2]= new Solenoid(7);
		soy[3]= new Solenoid(8);
		piston_position = 0;
		grab_position = 0;
		minibot_deployed = 0;
		forkliftjag = new Jaguar(5);
		flexjag = new Jaguar(6);

		myRobot->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		myRobot->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);

		myRobot->SetExpiration(0.1);
		SmartDashboard::init();
		SmartDashboard::Log("initializing...", "System State");

		//camera->WriteBrightness(0);

		teleCameraTask = new Task("TeleCamera",(FUNCPTR)&TeleCamera);
		cameraTask = new Task("Camerastuff",(FUNCPTR)&AutoCamera);		
		compressor->Start();
		//flexTask = new Task("flex_task",(FUNCPTR)&flex_task);
		//flex_direction = 0;
	}

	/**
	 * Drive left & right motors for 1 seconds then stop
	 */
	void Autonomous(void)
	{
		led_relay->Set(Relay::kOn);
		SmartDashboard::Log("ON", "LED");
		SmartDashboard::Log("Autonomous", "System State");
		myRobot->SetSafetyEnabled(false);
		SmartDashboard::Log("Foward", "Autodrive state");
		hand_grab();
		SmartDashboard::Log("Stop", "Autodrive state");
		myRobot->MecanumDrive_Polar(0.7, 0.0, -0.01);  	 //speed, direction (degrees), rotation (-1..1)
		Wait(1.9); // Drive for X.x seconds
		myRobot->MecanumDrive_Polar(0.0, 0, 0.0); 	// stop robot
		SmartDashboard::Log("Camera", "Autodrive state");
		forkliftjag->Set(-0.7); //reverse, so pulling the stick back goes up
		Wait(2.7);
		forkliftjag->Set(0.0);
		cameraTask->Start((UINT32)myRobot); // Drive by camera!
		Wait(4.0);
		cameraTask->Stop();
		myRobot->MecanumDrive_Polar(0.4, 0.0, 0.0); // drive a bit forward
		Wait(0.85);
		myRobot->MecanumDrive_Polar(0.0, 0.0, 0.0); // stop
		Wait(0.5);
		hand_release();
		forkliftjag->Set(0.2);
		Wait(0.45);
		forkliftjag->Set(0.0);
		Wait(0.5);
		myRobot->MecanumDrive_Polar(-0.4, 0.0, 0.0); // drive a bit backward
		Wait(0.90);
		myRobot->MecanumDrive_Polar(0.0, 0.0, 0.0); // stop

	}
	
	static void flex_task(UINT32 _my_robot)
	{
		/*RobotDemo *my_robot=(RobotDemo*)_my_robot;
		SmartDashboard::Log(my_robot->armstick->GetZ(), "Flex Task Z");
		SmartDashboard::Log(my_robot->flex_direction, "Flex Task Rotation");
		if(my_robot->flex_direction > 0){
			my_robot->flexjag->Set(0.4);
		}
		else if(my_robot->flex_direction < 0){
			my_robot->flexjag->Set(-0.4);
		}else{
			my_robot->flexjag->Set(0);
		}
		*/
	}

	static void TeleCamera()
	{
		AxisCamera *camera; //Cameralol (:
		camera = &AxisCamera::GetInstance();
		camera->WriteResolution(AxisCameraParams::kResolution_160x120);
	}
	
	static void AutoCamera(RobotDrive *myRobot) 
	{
		AxisCamera *camera; //Cameralol (: 
		HSLImage *hslimage;
		//ColorImage *cImage;
		vector<ParticleAnalysisReport>* pars;
		//Threshold tapeThreshold(43, 44, 250, 255, 96, 255);
		//Threshold tapeThreshold(22, 24, 41, 45, 99, 100);//hsv
		//Threshold tapeThreshold(30, 60, 40, 80, 220, 255); //red hsl
		Threshold tapeThreshold(26, 253, 0, 10, 220, 255); //red hsl as of 20110303
		BinaryImage *tapePixels;
		ParticleAnalysisReport par;
		int driving = 0;
		double right_driveSpeed = 0.8; // speed in given direction
		double left_driveSpeed = 0.5; // speed in given direction
		double driveDuration = 0.07; // seconds of drive time
		double totalDriveTime = 2.0; //total drive time in camera mode
		double dt = 0.0; // drive time accumulator
		double xOffset = 0.05; // X Offset left of center
		
		camera = &AxisCamera::GetInstance();
		camera->WriteResolution(AxisCameraParams::kResolution_160x120);
		SmartDashboard::Log("before while", "CameraTask");
		while (dt <= totalDriveTime ) {
			SmartDashboard::Log(dt, "Drivetime");
			SmartDashboard::Log(driving, "driving?");
			if( driving ) {
				//forkliftjag->Set(-0.15); //reverse, so pulling the stick back goes up
				Wait(driveDuration);
			//	forkliftjag->Set(0.0);
				dt += driveDuration;
				myRobot->MecanumDrive_Polar(0.0, 0.0, 0.0);
				driving = 0;
			}
			SmartDashboard::Log("GetImage","CameraTask");
			hslimage = camera->GetImage();

			tapePixels = hslimage->ThresholdHSL(tapeThreshold);
			pars = tapePixels->GetOrderedParticleAnalysisReports();
			if (pars->size() > 0) {
				par = (*pars)[0];
				SmartDashboard::Log(par.center_mass_x_normalized, "center of mass x");
				SmartDashboard::Log(par.center_mass_y_normalized, "center of mass y");
			}

			if (pars->size() > 0) {
			//	double closest_x_val = (*pars)[0].center_mass_x_normalized;
				double closest_x_val = 20.0;
				for(unsigned int i=0; i < pars->size(); i++) {
					par = (*pars)[i];
					if (fabs(par.center_mass_x_normalized)
							< fabs(closest_x_val) && par.particleToImagePercent
							> 0.0001) {
						closest_x_val = par.center_mass_x_normalized;
					}
				}
				closest_x_val -= xOffset; // Because camera is not centered on the robot
				
				SmartDashboard::Log(closest_x_val, "closest x val");
				if (closest_x_val > 0.005) {
					//we're too far to the left (anti-clockwise)
					SmartDashboard::Log("right", "which way");
					//Wait(1.5);
					myRobot->MecanumDrive_Polar(right_driveSpeed, 0.0, 0.01);//speed, direction (degrees), rotation (-1..1)
					driving = 1;
				} else if (closest_x_val < -0.005) {
					//too far right (clockwise)
					SmartDashboard::Log("left", "which way");
					//Wait(1.5);
					myRobot->MecanumDrive_Polar(left_driveSpeed, 0.0, -0.0);
					driving = 1;
				} else {
					SmartDashboard::Log("straight", "which way");
					//uh, we're already centered
					//Wait(1.5);
					myRobot->MecanumDrive_Polar(right_driveSpeed, 0.0, 0.0);
					driving = 1;
				}
			}
			pars->clear();
			delete pars;

			delete tapePixels;
			delete hslimage;
		}
		myRobot->MecanumDrive_Polar(0.0, 0, 0.0);
	}

	// Scale the inputs from the joysticks. This function assumes a lot
	// for example, the input should be from -1 to 1, the pwr value should
	// be a positive double - this function is not defensively programmed :-(
	double JoyStickScale(double jsInput, double cl, double pwr) {
		if( fabs(jsInput) <= cl ) {
				return 0.0; // input is less than control limit, so it's zero
		}
		return pow(fabs(jsInput), pwr) * jsInput/fabs(jsInput);
	}
	
	// Main operator control function
	void OperatorControl(void) {
		SmartDashboard::Log("Teleoperated", "System State");
		myRobot->SetSafetyEnabled(true);

			teleCameraTask->Start();
		//cameraTask->Start((UINT32)myRobot);
		//flexTask->Start((UINT32)this);

		while (IsOperatorControl()) {

			//myRobot->TankDrive(leftstick, rightstick);
			SmartDashboard::Log(rightstick->GetMagnitude(), "right magnitude");
			SmartDashboard::Log(rightstick->GetDirectionDegrees(), "right degrees");
			SmartDashboard::Log(leftstick->GetX(), "left getx");
			myRobot->MecanumDrive_Polar(JoyStickScale(rightstick->GetMagnitude(), 0.2, 3.5),
					rightstick->GetDirectionDegrees(), JoyStickScale(leftstick->GetX(), 0.2, 3.5));
			//	myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
			Wait(0.005); // wait for a motor update time


			if (armstick->GetRawButton(1)) //Tells us if the trigger is being pressed on the armstick
			{
				SmartDashboard::Log("Yes", "Trigger Pressed");
				//k_relay->Set(Relay::kOn);

			} else {
				SmartDashboard::Log("No", "Trigger Pressed");
				//k_relay->Set(Relay::kOff);
			}

			if (armstick->GetTop())  //Tells us if the top button is being pressed on the armstick
			{
				SmartDashboard::Log("Yes", "Top Pressed");
				//k_relay->Set(Relay::kOn);

			} else {
				SmartDashboard::Log("No", "Top Pressed");
				//k_relay->Set(Relay::kOff);
			}

			SmartDashboard::Log(armstick->GetX(), "Armstick X");
			SmartDashboard::Log(armstick->GetY(), "Armstick Y");
			

			if (fabs(armstick->GetY()) > 0.25) {
				forkliftjag->Set(armstick->GetY()/-1.5); //reverse, so pulling the stick back goes up
			}else{
				forkliftjag->Set(0);
			}

			if (armstick->GetZ() > 0.9) {//down
				//flex_direction = 1;
				flexjag->Set(0.25);
			}
			else if (armstick->GetZ() < -0.9){//up
				//flex_direction = -1;
				flexjag->Set(-0.6);
			}else{
				//flex_direction = 0;
				flexjag->Set(0);
			}

			DriverStationEnhancedIO &controller_box =
					DriverStation::GetInstance()->GetEnhancedIO();
			if (controller_box.GetDigital(3)) {
				compressor->Start();
				controller_box.SetDigitalOutput(9,1); // digital i/o 0 on enhanced i/o 
				SmartDashboard::Log("ON", "compressor");
				//starts compressor when switch 3 flicked
			} else {
				compressor->Stop();
				controller_box.SetDigitalOutput(9,0); // digital i/o 0 on enhanced i/o
				SmartDashboard::Log("OFF", "compressor");
			}

			if (controller_box.GetDigital(4)) {
				SmartDashboard::Log("ON", "LED");
				led_relay->Set(Relay::kOn);
				controller_box.SetDigitalOutput(10,1); // digital i/o 1 on enhanced i/o 
			} else {
				SmartDashboard::Log("OFF", "LED");
				led_relay->Set(Relay::kOff);
				controller_box.SetDigitalOutput(10,0); // digital i/o 1 on enhanced i/o 
			}
			
			if (controller_box.GetDigital(7)) 
			{
				if(!minibot_deployed){
					soy[2]->Set(true);
					Wait(0.03);
					soy[2]->Set(false);
					minibot_deployed = 1;
				}
				//Release the Mini bot!!!!!!!!!!!
			}
			else 
			{
				if (minibot_deployed) {
					soy[3]->Set(true);
					Wait(0.03);
					soy[3]->Set(false);
					minibot_deployed = 0;
				}
			}


			if (armstick->GetTrigger()) //4 is the Solenoid switcherrooo thinggyyy 
			{
				hand_grab();
				SmartDashboard::Log("Grab", "Hand");
				//makes piston go up
			} 
			if(armstick->GetTop()){
				hand_release();
				SmartDashboard::Log("Release", "Hand");
				//makes piston go down down down down down 
			}
			

		}

	}

	
	void hand_grab(void) 
	{
		if (piston_position == 0) {
			soy[0]->Set(false);
			soy[1]->Set(true);
			Wait(0.03);
			soy[0]->Set(false);
			soy[1]->Set(false);
			piston_position = 1;
		}
	}

	void hand_release(void) 
	{
		if (piston_position == 1) {
			soy[0]->Set(true);
			soy[1]->Set(false);
			Wait(0.03);
			soy[0]->Set(false);
			soy[1]->Set(false);
			piston_position = 0;
		}

	}

};

START_ROBOT_CLASS(RobotDemo)
;

