/*
 * HelloPi.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: raspberry
 */

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <pigpio.h>
#include <iostream>
#include <sstream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#define GPIOSTEERINGCONTROL 4  //pin 7
#define GPIOMOTORCONTROL 	17    //pin 11
#define MAXSTEERINGOFFSET 	200
#define MAXMOTOROFFSET 		200
#define SERVOCENTER			1500

using namespace std;
using boost::property_tree::ptree;


typedef websocketpp::server<websocketpp::config::asio> server;

void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
        std::cout << msg->get_payload() << std::endl;

        ptree parsedJson;
    	stringstream jsonStream;
    	jsonStream << msg->get_payload();
    	read_json(jsonStream, parsedJson);

    	int servoInt = (int)(SERVOCENTER + ( parsedJson.get<double>("leftAnalog_x") * MAXSTEERINGOFFSET ));
    	int motorInt = (int)(SERVOCENTER + ( parsedJson.get<double>("rightAnalog_y") * MAXMOTOROFFSET ));
    	gpioServo(GPIOSTEERINGCONTROL,servoInt);
    	gpioServo(GPIOMOTORCONTROL, motorInt);




        //std::istringstream(msg->get_payload()) >> servoInt;
        //gpioServo(GPIOsteeringControl, servoInt);
        //gpioServo(GPIOmotorControl, servoInt);
}

void GPIOcallBack(int gpio, int level, uint32_t tick){
	/* configure library */
	//cout << "\tPWM callback fired level: " << level << endl;

}

int main() {

    int GPIOstatus;
    int GPIOfrequencySteering;
    int GPIOfrequencyMotor;
    GPIOstatus = gpioInitialise();

    if (GPIOstatus < 0)
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

    gpioServo(GPIOSTEERINGCONTROL, 1500);

    //gpioPWM(GPIO, 128);

    //Start the websocket server
    server print_server;
    print_server.set_message_handler(&on_message);

    print_server.init_asio();
    print_server.listen( boost::asio::ip::tcp::v4() , 8080 );
    print_server.start_accept();
    cout << "Starting RoadStar server" << endl;
    print_server.run();


    gpioTerminate();

    return 0;
}
