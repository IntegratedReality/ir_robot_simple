#pragma once

#include <iostream>
#include <cstring>
#include <oscpack/osc/OscReceivedElements.h>
#include <oscpack/osc/OscPacketListener.h>
#include <oscpack/ip/UdpSocket.h>

#include "../common/RobotData.h"
#include "../common/Operation.h"

class RobotListener : public osc::OscPacketListener {
	public:
		RobotListener(RobotData* _data, permsAry* _permissions, double* _right, double* _left);
		virtual void ProcessMessage(const osc::ReceivedMessage& msg, const IpEndpointName& remoteEndPoint );
		bool checkMessageReceived(void);
	private:
		RobotData* data;
    permsAry* permissions;
		double* Duty_right;
		double* Duty_left;
		//ETeam *owner;
		bool m_message_received;
};
