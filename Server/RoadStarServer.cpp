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


#define GPIOSTEERINGCONTROL 4  //pin 7
#define GPIOMOTORCONTROL 	17    //pin 11
#define MAXSTEERINGOFFSET 	300
#define MAXMOTOROFFSET 		250
#define MAXMOTORCREEPEROFFSET 150
#define SERVOCENTER			1550
#define NONEWMESSAGE		2
using namespace std;
using boost::property_tree::ptree;

struct state
{
	bool connected;
	bool creeper;
	bool cruse;
	bool leftTurn;
	bool rightTurn;
	int  turnRadious;
	int  servoInt;
	int  motorInt;
	int  speed;
	int  distanceTraveled;
	unsigned long lastMessageTime;
};
using websocketpp::connection_hdl;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;
typedef websocketpp::server<websocketpp::config::asio> server;
//typedef std::set<connection_hdl,std::owner_less<connection_hdl>> con_list;

state currentState;
server websocket_server;
//con_list websocket_connections;
pthread_t watchDog_th;
pthread_mutex_t watchDog_mutex = PTHREAD_MUTEX_INITIALIZER;



void  * watchDog(void * argument);
void on_message(websocketpp::connection_hdl, server::message_ptr);
//void on_open(websocketpp::connection_hdl hdl);


int main() {
	////////////////////////init current state////////////////
	currentState.connected   = false;
	currentState.creeper     = false;
	currentState.cruse 	     = false;
	currentState.leftTurn    = false;
	currentState.rightTurn   = false;
	currentState.turnRadious = 0;
	currentState.servoInt    = 0;
	currentState.motorInt    = 0;
	currentState.speed		 = 0;
	currentState.lastMessageTime = 0;
	currentState.distanceTraveled = 0;

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
    gpioSetPWMrange(GPIOSTEERINGCONTROL, 255);
    gpioSetPWMfrequency(GPIOSTEERINGCONTROL, 0);
    GPIOfrequencySteering = gpioGetPWMfrequency(GPIOSTEERINGCONTROL);

    //motor
    gpioSetPWMrange(GPIOMOTORCONTROL, 255);
    gpioSetPWMfrequency(GPIOMOTORCONTROL, 0);
    GPIOfrequencyMotor = gpioGetPWMfrequency(GPIOMOTORCONTROL);
    cout << "FREQ: Using " << GPIOfrequencySteering << " for steering and "
    		<< GPIOfrequencyMotor << " for motor." << endl;
    //gpioSetAlertFunc(GPIOsteeringControl, GPIOcallBack);
    gpioServo(GPIOSTEERINGCONTROL, SERVOCENTER);		//set steering to straight

    /////////////////////////watchdog thread////////////////////////////

    pthread_create( &watchDog_th, NULL, watchDog, NULL);

    ////////////////////////Websocket Server setup////////////////////////

    websocket_server.set_message_handler(&on_message);			//what function should be called on a new message
    websocket_server.set_open_handler(&on_open);			//what function should be called on a new message
    websocket_server.init_asio();
    websocket_server.listen( boost::asio::ip::tcp::v4() , 8080 ); //What ip version and port should we listen on
    websocket_server.start_accept();
    cout << "Starting RoadStar server" << endl;
    websocket_server.run();										//start the server

    gpioTerminate();

    return 0;
}


void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {

        try{
        	//websocket_server.send("TIME");

        	//parse json to ptree for easy reading


            ptree parsedJson;
        	stringstream jsonStream;
        	jsonStream << msg->get_payload();
        	read_json(jsonStream, parsedJson);

        	ptree answerJson;
        	stringstream jsonWriteStream;
        	answerJson.put("speed", currentState.speed);
        	answerJson.put("media", false);
        	answerJson.put("distanceTraveled", currentState.distanceTraveled);
        	write_json(jsonWriteStream, answerJson);
        	//websocket_server.send(jsonWriteStream);


        	currentState.servoInt = (int)(SERVOCENTER +
        			( parsedJson.get<double>("leftAnalog_x") * MAXSTEERINGOFFSET ));

        	if(currentState.creeper == true){
        		currentState.motorInt = (int)(SERVOCENTER +
        				( parsedJson.get<double>("rightAnalog_y") * MAXMOTORCREEPEROFFSET ) );

        	}else{
        		currentState.motorInt = (int)(SERVOCENTER +
        				( parsedJson.get<double>("rightAnalog_y") * MAXMOTOROFFSET ));
        	}



        	gpioServo(GPIOMOTORCONTROL, currentState.motorInt);
        	gpioServo(GPIOSTEERINGCONTROL,currentState.servoInt);

        	//toggle creeper mode
        	if(parsedJson.get<bool>("start")){
        		if(currentState.creeper == true){
        			currentState.creeper = false;
        			cout << "Creeper mode off" << endl;
        		}else{
        			currentState.creeper = true;
        			cout << "Creeper mode on" << endl;
        		}
        	}

        	//keep track of message recieved times.
			pthread_mutex_lock( &watchDog_mutex );
			currentState.lastMessageTime = time(NULL);
			pthread_mutex_unlock( &watchDog_mutex );

        }
        catch(std::exception const& e){
        	cerr << e.what() << endl;
        	cerr << msg->get_payload() << std::endl;
        }

}

//void on_open(connection_hdl hdl) {
//    m_connections.insert(hdl);
//}

void * watchDog(void * argument){

	while(true){
		cout << "hello from the watchDog" << endl;
		if(currentState.connected == true){
			pthread_mutex_lock( &watchDog_mutex );
			if( (time(NULL) - currentState.lastMessageTime) > NONEWMESSAGE){
				//STOP!
				gpioServo(GPIOMOTORCONTROL, SERVOCENTER);
			}
			pthread_mutex_unlock( &watchDog_mutex );
		}

		sleep(1);
	}

	return 0;
}

void GPIOcallBack(int gpio, int level, uint32_t tick){
	/* configure library */
	//cout << "\tPWM callback fired level: " << level << endl;

}
