/*
 * HelloPi.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: raspberry
 */

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <pigpio.h>
#include <set>
#include <iostream>
#include <sstream>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <unistd.h>
#include <linux/input.h>
#include <fcntl.h>

#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#define GPIO_STEERING_CONTROL 4  //pin 7
#define GPIO_MOTOR_CONTROL 	  17 //pin 11
#define GPIO_RIGHT_TURN_SIGNAL 27//pin 13
#define GPIO_LEFT_TURN_SIGNAL  22//pin 15
#define MAX_STEERING_OFFSET 	270
#define TURN_THRESHOLD			75
#define MOTOR_FORWARD_RANGE 		40
//#define MOTOR_FORWARD_START			1620
#define MOTOR_FORWARD_START			1618
#define MOTOR_BACKWARD_RANGE 		100
#define MOTOR_BACKWARD_START		1490
#define MAX_MOTOR_CREEPER_OFFSET 150
#define ACCELERATION_THRESHOLD 1
#define SERVO_CENTER			1550
#define NO_NEW_MESSAGE		1
#define MOUSE_FILE "/dev/input/event0"
#define MOUSE_MM_PER_CLICK 4.444
#define MOUSE_SAMPLE_RATE	250000000
#define TURN_SIGNAL_RATE	500000000
#define SONAR_SAMPLE_RATE	250000000
#define COLLISION_THRESHOLD 40
#define STOP_WAIT			10000000
#define STOP_SPEED			1470

using namespace std;
using boost::property_tree::ptree;

struct state
{
	bool connected;
	bool autonomous;
	bool cruise;
	bool leftTurn;
	bool rightTurn;
	bool turnPeaked;
	bool signalOn;
	int  servoInt;
	int  motorInt;
	int  lastRequestedMotorInt;
	float  speed;
	float  distanceTraveled;
	unsigned long lastMessageTime;
	websocketpp::connection_hdl client;
	bool obstical;
	int  sonarRange;
};

typedef websocketpp::server<websocketpp::config::asio> server;
using websocketpp::connection_hdl;


state currentState;
server websocket_server;

pthread_t watchDog_th;
pthread_t turnSignal_th;
pthread_t speedometer_th;
pthread_mutex_t watchDog_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t turnSignal_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t speedometer_mutex = PTHREAD_MUTEX_INITIALIZER;

//sonar
pthread_t sonar_th;
void * sonar(void * argument);
pthread_mutex_t sonarRange_mutex = PTHREAD_MUTEX_INITIALIZER;
unsigned int sonarRange = 1100;		//initialized to full range

void  * watchDog(void * argument);
void  * turnSignal(void * argument);
void  * speedometer(void * argument);
void on_message(websocketpp::connection_hdl, server::message_ptr);
void on_open(websocketpp::connection_hdl hdl);
void on_close(websocketpp::connection_hdl hdl);


int main() {
	////////////////////////init current state////////////////
	currentState.connected   = false;
	currentState.autonomous  = false;
	currentState.cruise 	 = false;
	currentState.leftTurn    = false;
	currentState.rightTurn   = false;
	currentState.turnPeaked	 = false;
	currentState.signalOn    = false;
	currentState.obstical	 = false;
	currentState.servoInt    = 0;
	currentState.motorInt    = 0;
	currentState.speed		 = 0;
	currentState.lastMessageTime = 0;
	currentState.distanceTraveled = 0;
	currentState.lastRequestedMotorInt = 0;
	currentState.sonarRange   = 200;
	////////////////////////GPIO setup////////////////////////
    int GPIOstatus;
    int GPIOfrequencySteering;
    int GPIOfrequencyMotor;
    GPIOstatus = gpioInitialise();				//required for GPIO to function

    if (GPIOstatus < 0)							//did we succeed
    {
       fprintf(stderr, "pigpio initialisation failed.\n");
       return 1;
    }


    //steering
    gpioSetPWMrange(GPIO_STEERING_CONTROL, 255);
    gpioSetPWMfrequency(GPIO_STEERING_CONTROL, 0);
    GPIOfrequencySteering = gpioGetPWMfrequency(GPIO_STEERING_CONTROL);
    int GPIOrangeSteering = gpioGetPWMrange(GPIO_MOTOR_CONTROL);
    //motor
    gpioSetPWMrange(GPIO_MOTOR_CONTROL, 255);
    gpioSetPWMfrequency(GPIO_MOTOR_CONTROL, 0);
    int GPIOrangeMotor = gpioGetPWMrange(GPIO_MOTOR_CONTROL);
    GPIOfrequencyMotor = gpioGetPWMfrequency(GPIO_MOTOR_CONTROL);
    cout << "FREQ: Using " << GPIOfrequencySteering << " for steering and "
    		<< GPIOfrequencyMotor << " for motor." << endl;
    cout << "RANGE: Using " << GPIOrangeSteering << " for steering and "
    		<< GPIOrangeMotor << "for motor." << endl;
    //gpioSetAlertFunc(GPIOsteeringControl, GPIOcallBack);
    gpioServo(GPIO_STEERING_CONTROL, SERVO_CENTER);		//set steering to straight
    //set mode of turn signals to output
    gpioSetMode(GPIO_RIGHT_TURN_SIGNAL, PI_OUTPUT);
    gpioSetMode(GPIO_LEFT_TURN_SIGNAL, PI_OUTPUT);

    /////////////////////////threads////////////////////////////
    pthread_create( &watchDog_th, NULL, watchDog, NULL);
    pthread_create( &turnSignal_th, NULL, turnSignal, NULL);
    pthread_create( &sonar_th, NULL, sonar, NULL);
    pthread_create( &speedometer_th, NULL, speedometer, NULL);

    ////////////////////////Websocket Server setup////////////////////////
    //what function should be called on a new message
    websocket_server.set_message_handler(&on_message);			//what function should be called on a new message
    websocket_server.set_open_handler(&on_open);
    websocket_server.set_close_handler(&on_close);
    websocket_server.init_asio();
    //What ip version and port should we listen on
    websocket_server.listen( boost::asio::ip::tcp::v4() , 8080 );
    websocket_server.start_accept();
    //Way to much log
    websocket_server.clear_access_channels(websocketpp::log::alevel::all);
    cout << "Starting RoadStar server" << endl;
    websocket_server.run();										//start the server

    gpioTerminate();

    return 0;
}

void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
	try{
		//immediatly send back a message for timing purposes
		//websocket_server.send(hdl ,"TIME",websocketpp::frame::opcode::text);



		//parse the json
		ptree parsedJson;
		stringstream jsonStream;
		jsonStream << msg->get_payload();
		read_json(jsonStream, parsedJson);



		//send back the relevent status;

//		ptree answerJson;
//		stringstream jsonWriteStream;
//		pthread_mutex_lock( &speedometer_mutex );
//		answerJson.put("speed", currentState.speed);
//		answerJson.put("distanceTraveled", currentState.distanceTraveled);
//		if(currentState.distanceTraveled != 0){
//			currentState.distanceTraveled = 0; 		//only send a distance traveled once
//		}
//		pthread_mutex_unlock( &speedometer_mutex );
//		write_json(jsonWriteStream, answerJson);
//		websocket_server.send(hdl,jsonWriteStream.str(),websocketpp::frame::opcode::text);

		//toggle autonomous mode
		if(parsedJson.get<bool>("start")){
			if(currentState.autonomous == true){
				currentState.autonomous = false;
				cout << "Autonomous mode off." << endl;
			}else{
				currentState.autonomous = true;
				cout << "Autonomous mode on." << endl;
			}
		}

		if(currentState.autonomous != true){

			//update the servo values
			currentState.servoInt = (int)(SERVO_CENTER +
					( parsedJson.get<double>("leftAnalog_x") * MAX_STEERING_OFFSET ));

			if(currentState.cruise == false ){
				//skip the center range
				int requestedMotorInt = 0;
				if( (parsedJson.get<double>("rightAnalog_y") > 0) && (currentState.obstical != true) ){
					//forward
					//cout << "rightAnalog_y: " << parsedJson.get<double>("rightAnalog_y") << endl;
					requestedMotorInt = (int)( parsedJson.get<double>("rightAnalog_y") * MOTOR_FORWARD_RANGE );
					//cout << "requestedMotirInt(forward): " << requestedMotorInt << endl;
					if((requestedMotorInt - currentState.lastRequestedMotorInt) > ACCELERATION_THRESHOLD){
						requestedMotorInt = currentState.lastRequestedMotorInt + ACCELERATION_THRESHOLD;
						//cout << "Don't accelerate so quickly!" << endl;
					}
					currentState.motorInt = MOTOR_FORWARD_START  + requestedMotorInt;

				}else if ( parsedJson.get<double>("rightAnalog_y") < 0){
					//backward
					//cout << "rightAnalog_y: " << parsedJson.get<double>("rightAnalog_y") << endl;
					requestedMotorInt = -1*(int)( parsedJson.get<double>("rightAnalog_y") * MOTOR_BACKWARD_RANGE );
					//cout << "requestedMotirInt(backward): " << requestedMotorInt << endl;
					if((currentState.lastRequestedMotorInt - requestedMotorInt) > ACCELERATION_THRESHOLD){
						requestedMotorInt = currentState.lastRequestedMotorInt - ACCELERATION_THRESHOLD;
						//cout << "Don't accelerate (backwards) so quickly!" << endl;
					}
					currentState.motorInt = MOTOR_BACKWARD_START - requestedMotorInt;
					if(currentState.cruise){
						currentState.cruise = false;
					//	cout << "Cruise mode off." << endl;
					}

				}else{
					//stop
					currentState.motorInt = SERVO_CENTER;
				}

				//cout << "updating lastRequestedMotorInt: " << currentState.lastRequestedMotorInt << endl;
				currentState.lastRequestedMotorInt = requestedMotorInt;

				//update motor
				//cout << "updating motor: " << currentState.motorInt << endl;
				gpioServo(GPIO_MOTOR_CONTROL, currentState.motorInt);
			}
			//update servo
			gpioServo(GPIO_STEERING_CONTROL,currentState.servoInt);



			//toggle turn signal
			//right
			pthread_mutex_lock( &turnSignal_mutex );
			if(parsedJson.get<bool>("R1")){
				if(currentState.rightTurn == true){
					currentState.rightTurn = false;
					cout << "Right turn signal off." << endl;
				}else if(currentState.leftTurn == true){
					currentState.leftTurn = false;
					currentState.rightTurn = true;
					cout << "Left turn signal off." << endl;
					cout << "Right turn signal on." << endl;
				}
				else{
					currentState.rightTurn = true;
					currentState.leftTurn = false;
					cout << "Right turn signal on." << endl;
				}
			}
			//left
			if(parsedJson.get<bool>("L1")){
				if(currentState.leftTurn == true){
					currentState.leftTurn = false;
					cout << "Left turn signal off." << endl;
				}else if(currentState.rightTurn == true){
					currentState.rightTurn = false;
					currentState.leftTurn = true;
					cout << "Right turn signal off." << endl;
					cout << "Left turn signal on." << endl;
				}else{
					currentState.leftTurn = true;
					currentState.rightTurn = false;
					cout << "Left turn signal on." << endl;
				}
			}

			//check for peak
			if(currentState.leftTurn || currentState.rightTurn){
				if(currentState.turnPeaked){
					if(currentState.servoInt == SERVO_CENTER){
						currentState.leftTurn = false;
						currentState.rightTurn = false;
						currentState.turnPeaked = false;
						cout << "Turn complete" << endl;
					}
				}else{
					if(currentState.leftTurn){
						if(currentState.servoInt < (SERVO_CENTER - TURN_THRESHOLD)){
							currentState.turnPeaked = true;
						}
					}else if(currentState.rightTurn){
						if(currentState.servoInt > (SERVO_CENTER + TURN_THRESHOLD)){
							currentState.turnPeaked = true;
						}
					}
				}
			}
			pthread_mutex_unlock( &turnSignal_mutex );



			//toggle cruise mode
			if(parsedJson.get<bool>("left")){
				if(currentState.cruise == true){
					currentState.cruise = false;
					cout << "Cruise mode off" << endl;
				}else{
					currentState.cruise = true;
					cout << "Cruise mode on" << endl;
				}
			}
			if(parsedJson.get<bool>("up")){
				currentState.motorInt = currentState.motorInt + 10;
				gpioServo(GPIO_MOTOR_CONTROL, currentState.motorInt);
			}else if(parsedJson.get<bool>("down")){
				currentState.motorInt = currentState.motorInt - 10;
				gpioServo(GPIO_MOTOR_CONTROL, currentState.motorInt);
			}

		}







		//keep track of message recieved times.
		pthread_mutex_lock( &watchDog_mutex );
		currentState.lastMessageTime = time(NULL);
		currentState.connected = true;
		pthread_mutex_unlock( &watchDog_mutex );


	}
	catch(std::exception const& e){
		cerr << "<ERROR>" << endl;
		cerr << e.what() << endl;
		cerr << msg->get_payload() << endl;
		cerr << "</ERROR>" << endl;
	}

}

void * sonar(void * arguments){
	const timespec waitTime = { 0, SONAR_SAMPLE_RATE };
	const timespec waitTimeStop = { 0, STOP_WAIT };
	int fd;														// File descrition
	const char *fileName = "/dev/i2c-1";						// Name of the port we will be using
	int  address = 0x70;										// Address of the SRF02 shifted right one bit
	unsigned char buf[10];										// Buffer for data being read/ written on the i2c bus
	unsigned char readBuf[4];

	if ((fd = open(fileName, O_RDWR)) < 0) {					// Open port for reading and writing
		cout << "Failed to open i2c port" << endl;
	}

	if (ioctl(fd, I2C_SLAVE, address) < 0) {					// Set the port options and set the address of the device we wish to speak to
		cout << "Unable to get bus access to talk to slave" << endl;
	}

	buf[0] = 1;		// Commands for performing a ranging
	buf[1] = 8;	//return distance in
	if ((write(fd, buf, 2)) != 2) {	// Write commands to the i2c port
		cout << "Error writing gain" << endl;
	}

	while(true){
		//ping
		buf[0] = 0;		// Commands for performing a ranging
		buf[1] = 81;	//return distance in
		if ((write(fd, buf, 2)) != 2) {	// Write commands to the i2c port
			cout << "Error writing to slave" << endl;
		}

		nanosleep(&waitTime,NULL);

		buf[0] = 0;
		//get sonar data
		if ((write(fd, buf, 1)) != 1) {	// Write commands to the i2c port
			cout << "Error writing to slave" << endl;
		}
		if ((read(fd, readBuf, 4)) != 4) {	// Write commands to the i2c port
			cout << "Error reading high bit" << endl;
		}

		//set sonar range or perform operations

		sonarRange = readBuf[2];
		sonarRange = (sonarRange << 8) + readBuf[3];
		cout << "sonar range = " << sonarRange << endl;
		pthread_mutex_lock( &sonarRange_mutex );
		currentState.sonarRange = sonarRange;
		if(sonarRange < COLLISION_THRESHOLD){
			currentState.obstical = true;
			if(currentState.motorInt >= MOTOR_FORWARD_START){
				gpioServo(GPIO_MOTOR_CONTROL,STOP_SPEED);
				currentState.motorInt = STOP_SPEED;
				gpioServo(GPIO_MOTOR_CONTROL,STOP_SPEED);
				nanosleep(&waitTimeStop,NULL);
				gpioServo(GPIO_MOTOR_CONTROL,SERVO_CENTER);
				currentState.motorInt = SERVO_CENTER;
			}

			cout << "OBSTICAL!" << endl;
		}else{
			currentState.obstical = false;
		}
		pthread_mutex_unlock( &sonarRange_mutex );

	}

	return 0;
}

void * watchDog(void * argument){

	while(true){
		//cout << "hello from the watchDog" << endl;
		pthread_mutex_lock( &watchDog_mutex );
		if(currentState.connected == true){
			if( (time(NULL) - currentState.lastMessageTime) > NO_NEW_MESSAGE){
				//STOP!
				gpioServo(GPIO_MOTOR_CONTROL, SERVO_CENTER);
				cout << "EMERGENCY STOP!!! Signal lost." << endl;
			}
		}
		pthread_mutex_unlock( &watchDog_mutex );
		sleep(1);
	}

	return 0;
}

void * turnSignal(void * argument){
	const timespec waitTime = { 0, TURN_SIGNAL_RATE };
	while(true){
		pthread_mutex_lock( &turnSignal_mutex );
		if(currentState.rightTurn){
			if(currentState.signalOn){
				gpioWrite_Bits_0_31_Clear( (1<<GPIO_RIGHT_TURN_SIGNAL) );
				currentState.signalOn = false;
			}else{
				gpioWrite_Bits_0_31_Set( (1<<GPIO_RIGHT_TURN_SIGNAL) );
				currentState.signalOn = true;
			}
		}else if(currentState.leftTurn){
			if(currentState.signalOn){
				gpioWrite_Bits_0_31_Clear( (1<<GPIO_LEFT_TURN_SIGNAL) );
				currentState.signalOn = false;
			}else{
				gpioWrite_Bits_0_31_Set( (1<<GPIO_LEFT_TURN_SIGNAL) );
				currentState.signalOn = true;
			}
		}else{
			//if not making a right or left turn turn signals off
			gpioWrite_Bits_0_31_Clear( (1<<GPIO_LEFT_TURN_SIGNAL) | (1<<GPIO_RIGHT_TURN_SIGNAL));
		}
		pthread_mutex_unlock( &turnSignal_mutex );
		nanosleep(&waitTime,NULL);
	}

	return 0;
}

void * speedometer(void * argument){
	const timespec waitTime = { 0, MOUSE_SAMPLE_RATE };
	cout << waitTime.tv_nsec << " " << waitTime.tv_sec << endl;

	int fd;
    struct input_event ie;

    if((fd = open(MOUSE_FILE, O_RDONLY)) == -1) {
        perror("opening device");
   //     exit(1);
    }
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    int counter = 0;

	while(true){
		//cout << "hello from the speedometer" << endl;
		nanosleep(&waitTime,NULL);
		//cout << "wait over" << endl;
		counter = 0;

	    while(read(fd, &ie, sizeof(struct input_event)) != -1) {
	    	//cout << "reading mouse" << endl;
	    	if((ie.type == 2) && (ie.code == 8)){
	    		counter++;
	    	//	cout << "new mouse event " << counter << endl;
	    	}
	    }

	   //cout << "aquireing mutex" << endl;
		pthread_mutex_lock( &speedometer_mutex );

		//cout << "aquired mutex" << endl;
		currentState.speed = (counter * MOUSE_MM_PER_CLICK) * 2;   //mm/s
		currentState.distanceTraveled = counter * MOUSE_MM_PER_CLICK;				//mm.
		//cout << "speed: " << currentState.speed << " distance: " << currentState.distanceTraveled << endl;

		ptree answerJson;
		stringstream jsonWriteStream;

		answerJson.put("speed", currentState.speed);
		answerJson.put("distanceTraveled", currentState.distanceTraveled);
		write_json(jsonWriteStream, answerJson);
		//cout << "sending speedometer" << endl;
		if(currentState.connected){
			websocket_server.send(currentState.client,jsonWriteStream.str(),websocketpp::frame::opcode::text);

		}
		pthread_mutex_unlock( &speedometer_mutex );
		//cout << "released mutex" << endl;
	}

	return 0;
}

void on_open(websocketpp::connection_hdl hdl){
	cout << "Controller connected." << endl;
	currentState.connected = true;
	currentState.client = hdl;
	cout << "Client hdl saved." << endl;

}
void on_close(websocketpp::connection_hdl hdl){
	currentState.connected = false;
	cout << "Controller disconnected." << endl;
}

void GPIOcallBack(int gpio, int level, uint32_t tick){
	/* configure library */
	//cout << "\tPWM callback fired level: " << level << endl;

}
