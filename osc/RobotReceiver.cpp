#include "RobotReceiver.h"

void RobotReceiver::init() {
	//for	(int i = 0; i < NUM_OF_ROBOT; i++) {
		//data[i].init();
	//}
  data.init();
  for(auto& e: permissions){
    e = false;
  }

	//listener.setup(data, permissions);
  listener_ptr=std::unique_ptr<RobotListener>(new RobotListener(&data, &permissions, &Duty_right, &Duty_left));
  s = std::unique_ptr<UdpListeningReceiveSocket>
    (new UdpListeningReceiveSocket
     (IpEndpointName(IpEndpointName::ANY_ADDRESS, PORT_ROBOT), listener_ptr.get() )
     );
	std::thread th([this](){s -> RunUntilSigInt();});
	th.detach();


  //通信が来る前用のデフォルト値
  Duty_right = Duty_left = 1.0;
}

RobotData RobotReceiver::getData() {
	return data;
}

permsAry RobotReceiver::getPermissions(){
  return permissions;
}

/*
ETeam RobotReceiver::getPOOwner(int _id) {
	return owner[_id];
}
*/

bool RobotReceiver::checkMessageReceived(void)
{
	return listener_ptr -> checkMessageReceived();
}
