/*
 * HelloPi.cpp
 *
 *  Created on: Apr 15, 2014
 *      Author: raspberry
 */

#include <iostream>

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>

#include <pigpio.h>

#define GPIOsteeringControl 4  //pin 7
#define GPIOmotorControl 17    //pin 11

using namespace std;

typedef websocketpp::server<websocketpp::config::asio> server;

void on_message(websocketpp::connection_hdl hdl, server::message_ptr msg) {
        std::cout << msg->get_payload() << std::endl;
        int servoInt;
        std::istringstream(msg->get_payload()) >> servoInt;
        //gpioServo(GPIOsteeringControl, servoInt);
        gpioServo(GPIOmotorControl, servoInt);
}

void GPIOcallBack(int gpio, int level, uint32_t tick){
	/* configure library */
	//cout << "\tPWM callback fired level: " << level << endl;

}

int main() {
    cout << "Hello RPi Development World !"<< endl;
    cout << "Running servo test" << endl;

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
    gpioSetPWMrange(GPIOsteeringControl, 255);
    gpioSetPWMfrequency(GPIOsteeringControl, 0);
    GPIOfrequencySteering = gpioGetPWMfrequency(GPIOsteeringControl);

    //motor
    gpioSetPWMrange(GPIOmotorControl, 255);
    gpioSetPWMfrequency(GPIOmotorControl, 0);
    GPIOfrequencyMotor = gpioGetPWMfrequency(GPIOmotorControl);

    //gpioSetAlertFunc(GPIOsteeringControl, GPIOcallBack);

    gpioServo(GPIOsteeringControl, 1500);

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