#include "Sensor_data.h"
#include "Pose.h"

#include <cstdlib>
#include <ctime>
#include <thread>
#include <chrono>
#include <SFML/Graphics.hpp>
#include <iostream>
#include "kod/rplidar_sdk-master/sdk/sdk/include/rplidar.h"
#include "globala_variabler.h"

using namespace sf;
using namespace std;
using namespace rp::standalone::rplidar;

void * localization(void * threadid)
{

    //////////////////////GRAFIK///////////////////////////////
    
    RenderWindow SLAM_window(VideoMode(1024, 1024), L"RötSLAM");
    sf::View view{};
    view.zoom(8);
    view.setCenter(0,0);
    SLAM_window.setView(view);
    //////////////////////////////////////////////////////////
    struct timespec measure_start, measure_stop, slam_start, slam_stop,
	meas_pos_start, meas_pos_stop, main_stop, main_start;;
    Clock clock;
    Clock rplidar_measure_time;
    //Clock rplidar_measure_time;
    Time frame_delay_time {milliseconds(17)};

    Pose pose{0,0,0};
    float vel, steer_angle, frame_time;
    
    mtx.lock();
    float lr{glob_lr};
    float lf{glob_lf};
    mtx.unlock();

    // Skapar objekt av rplidarklass från rplidar_sdk
    RPlidarDriver* lidar = RPlidarDriver::CreateDriver();

    // söker efter en rplidar i "/dev/ttyUSB0"
    // om ingen hittas avbryts programmet
    u_result res{};
    res = lidar->connect("/dev/ttyUSB0",115200);
    if(!(IS_OK(res)))
    {
	cout << "Fel i kommunikation med RPLIDAR på /dev/ttyUSB0" << endl;
	res = lidar->connect("/dev/ttyUSB1",115200);
	if(!(IS_OK(res)))
	{
	    cout << "Fel i kommunikation med RPLIDAR på /dev/ttyUSB1. Ger upp..." << endl;
	    goto abort;
	}
	else
	{
	    cout << "Kommunikation med RPLIDAR på /dev/ttyUSB1 lyckades" << endl;
	}
    }
    else 
    {
	cout << "Kommunikation med RPLIDAR på /dev/ttyUSB0 lyckades" << endl;
    }

    // startar rplidarns motor och börjar skanna
    lidar->startMotor();
    lidar->startScan(0,1);
    
    clock_gettime(CLOCK_REALTIME, &slam_start);
    clock_gettime(CLOCK_REALTIME, &measure_start);
    clock_gettime(CLOCK_REALTIME, &meas_pos_stop);

    while (SLAM_window.isOpen())
    {
	//clock_gettime(CLOCK_REALTIME, &main_start);
	auto start_time = chrono::high_resolution_clock::now();

    /////////////////////GRAFIK////////////////////
	SLAM_window.clear();
	Event event;
	while(SLAM_window.pollEvent(event))
	{
	    if(event.type == Event::Closed)
	    {
		SLAM_window.close();
	    }
	}

	if (pose.sens_data.read_data(lidar))
	{
	    mtx.lock();
	    vel 	= glob_velocity;
	    steer_angle = glob_steer_angle;
	    mtx.unlock();
	    
	    clock_gettime(CLOCK_REALTIME, &measure_stop);
	    clock_gettime(CLOCK_REALTIME, &meas_pos_start);
	    pose.sens_data.convert_data(
		1000*((measure_stop.tv_sec - measure_start.tv_sec) + 
		(measure_stop.tv_nsec - measure_start.tv_nsec)/ 1000000000.0),
		1000*((meas_pos_start.tv_sec - meas_pos_stop.tv_sec) + 
		(meas_pos_start.tv_nsec - meas_pos_stop.tv_nsec)/ 1000000000.0),
		vel,steer_angle,lr,lf,SLAM_window);
	    clock_gettime(CLOCK_REALTIME, &measure_start);
	        
	    mtx.lock();
	    vel 	= glob_velocity;
	    steer_angle = glob_steer_angle;
	    mtx.unlock();
	    
	    clock_gettime(CLOCK_REALTIME, &slam_stop);
	    pose.SLAM(vel, steer_angle,
		1000*((slam_stop.tv_sec - slam_start.tv_sec) + 
		(slam_stop.tv_nsec - slam_start.tv_nsec)/ 1000000000.0),
		lr,lf);
	    clock_gettime(CLOCK_REALTIME, &slam_start);
	    clock_gettime(CLOCK_REALTIME, &meas_pos_stop);
	    
	    mtx.lock();
	    glob_landmarks     = pose.get_landmarks();
	    glob_landmark_info = pose.sens_data.get_landmark_info();
	    glob_x             = pose.get_x();
	    glob_y              = pose.get_y();
	    glob_ang           = pose.get_ang();
	    mtx.unlock();
	}
	//cur_data.convert_data(100,10000,301.69,42,42,SLAM_window);
	//cur_data.convert_data(100,10000,300.0,42,42,SLAM_window);
	//SLAM_test( SLAM_window);
	 SLAM_window.display();
	 
	 //clock_gettime(CLOCK_REALTIME, &main_stop);
	 auto end_time = chrono::high_resolution_clock::now();
	 //frame_time = 1000*((slam_stop.tv_sec - slam_start.tv_sec) + 
	 //	(slam_stop.tv_nsec - slam_start.tv_nsec)/ 1000000000.0);
	// cout << "clock: " << (clock.getElapsedTime()).asMicroseconds() << " gettime: " << 1000000*((stop.tv_sec - start.tv_sec) + (stop.tv_nsec - start.tv_nsec)/ 1000000000.0)<< endl;
	chrono::duration<float> elapsed_time{end_time - start_time};
	chrono::milliseconds elapsed_time_ms{chrono::duration_cast<chrono::milliseconds>(elapsed_time)};
	chrono::milliseconds delta{elapsed_time_ms - chrono::milliseconds{17}};
	
	if(delta.count() > 0)
	{
	    this_thread::sleep_for(delta);
	}
	
	// Avsluta om vi ska avsluta
        mtx.lock();
        bool exit{glob_exit};
        mtx.unlock();
        if (exit)
        {
            break;
        }
    }

abort:
    // Stannar och kopplar ifrån rplidarn
    lidar->stopMotor();
    lidar->disconnect();
    RPlidarDriver::DisposeDriver(lidar);
    pthread_exit(NULL);
}
