#pragma once

#include <iostream>
#include <string>
#include <thread>
#include <array>
#include <oscpack/osc/OscReceivedElements.h>
#include <oscpack/osc/OscPacketListener.h>
#include <oscpack/ip/UdpSocket.h>

#include "../common/RobotData.h"
#include "RobotListener.h"

const int PORT_ROBOT{8000};

class RobotReceiver {
	public:
		RobotReceiver() {}
		void init();
		void update();
		RobotData getData();
    	permsAry getPermissions();
		double getDutyRight() {return Duty_right;}
		double getDutyLeft() {return Duty_left;}
		//ETeam getPOOwner(int _id);
		bool checkMessageReceived(void);
	private:
		std::thread th;

    std::unique_ptr<RobotListener> listener_ptr;

    std::unique_ptr<UdpListeningReceiveSocket> s;
		RobotData data;
    permsAry permissions;
	double Duty_right;
	double Duty_left;
		//ETeam owner[3];
};
