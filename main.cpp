/*
   Robot's Main ProgramA of MRPM Project
   Dept. of Precision Engneering, U-tokyo
   Seimitsu Lab, Mayfes
 */

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <array>
#include <thread>
#include <mutex>
#include <eigen3/Eigen/Core>
#include "unistd.h"
#include "ledunit/ledunit.h"
#include "motor/robotcontrol.h"
#include "motor/drive.h"
#include "motor/params.h"
#include "osc/RobotReceiver.h"
//#include "osc/RobotSender.h"
//#include "AI/AI.h"
#include "wiringPi.h"

//const std::string mainServerHostName="Coconuts.local";
//const int PORT_MAINRCV=8000;

using namespace std;
std::mutex mutex_obj;

//extern int ID;

//void checkMovable(Position _pos, bool &_F, bool &_B);
//double distance(Position _p1, Position _p2);

//AI ai;

int main(int argc, char **argv)
{
  (void)argc;
  (void*)argv;
  /*
	if (argc == 2) ID = atoi(argv[1]);
	else {
		cout << "コマンドライン引数でロボットのIDを指定してください。m(__)m" << endl;
		return 0;
	}
  */

	//ai.init(ID);

	wiringPiSetupGpio();
	LedUnit led_unit(24);
	led_unit.on();

	RobotReceiver receiver;
	receiver.init();

	//RobotSender sender;
	//sender.setup(mainServerHostName, mainServerPort);

  //最初のメッセージが来るまでブロック
	while (!receiver.checkMessageReceived());

	DriveClass drive(MotorClass(22, 27, false),
      MotorClass(17, 18, true),
      receiver.getData().pos.x,
      receiver.getData().pos.y,
      receiver.getData().pos.theta,
      receiver.getData().time
      );

	std::thread robot_control_thread([&](){
		while (true) {
			while (!receiver.checkMessageReceived());
			mutex_obj.lock();
			drive.updateData(
          receiver.getData().pos.x,
          receiver.getData().pos.y,
          receiver.getData().pos.theta,
          receiver.getData().time
          );
			drive.updateDrive();
			//static bool last_shot_state = receiver.getData(ID).operation.shot;
      /*
			if (ID < 3 
          && !(receiver.getData().isAI ||
            receiver.getData().state == DEAD ||
            receiver.getData().state == STANDBY)
          ) {
				if (last_shot_state != receiver.getData(ID).operation.shot) {
					sender.sendShot(ID, receiver.getData(ID).operation.shot);
				}
			}
			last_shot_state = receiver.getData(ID).operation.shot;
      */
			mutex_obj.unlock();
		}
	});
	robot_control_thread.detach();

	long count = 0;
	/*
  std::thread ai_thread([&](){
		sender.sendShot(ID, false);
		while (ID >= 3 || receiver.getData(ID).isAI) {
			static bool last_shot_state = ai.getOperation().shot;
			if (!(receiver.getData(ID).state == DEAD ||
            receiver.getData(ID).state == STANDBY)
          ) {
				if (last_shot_state != ai.getOperation().shot) {
					sender.sendShot(ID, ai.getOperation().shot);
				}
			}
			last_shot_state = ai.getOperation().shot;
			for (int i = 0; i < 6; i++) {
      ai.setRobotData(i, receiver.getData(i));
			}
			for (int i = 0; i < 3; i++) {
			ai.setPOOwner(i, receiver.getPOOwner(i));
			}
			ai.update();
		}
	});
	ai_thread.detach();
  */




	while (1) {
    RobotData data;
    data = receiver.getData();
    permsAry permissions = receiver.getPermissions();

    //for debug
    /*
    std::cerr << "Permissions: ";
    for(auto& p:permissions){
      std::cerr << p << ", ";
    }
    std::cerr<<std::endl;
    */

    double max_v = 0.1, max_omega = 0.001;
    double v = 0, omega = 0;

    double ofsetRotate = 2.7;
    double ofsetMoving = 1.0;
    //bool brake{false};

    //動作許可を反映してトルク設定
    if (permissions[static_cast<size_t>(data.operation.direction)]){
      switch (data.operation.direction){
        case NO_INPUT:
          //brake = true;
          break;
        case TOP:
          v = max_v;
          break;
        case TOP_RIGHT:
          v = max_v;
          omega = -ofsetMoving * max_omega;
          break;
        case RIGHT:
          omega = - ofsetRotate * max_omega;
          break;
        case BOTTOM_RIGHT:
          v = -(max_v);
          omega = ofsetMoving * max_omega;
          break;
        case BOTTOM:
          v = -(max_v);
          break;
        case BOTTOM_LEFT:
          v = -(max_v);
          omega = -ofsetMoving * max_omega;
          break;
        case LEFT:
          omega = ofsetRotate * max_omega;
          break;
        case TOP_LEFT:
          v = max_v;
          omega = ofsetMoving * max_omega;
          break;
      }
    } else {
      //brake = true;
      //許可がない場合.
      //vやomegaはスコープ内で宣言してるから0になってくれるので放置でOK
    }

    mutex_obj.lock();
    drive.setTarget
      (v,
       omega,
       MotorMode::Move
       );
    mutex_obj.unlock();

    if (count != 3000) {
      count++;
      continue;
    }
    cout << "v: " << v << " ";
    cout << "omega: " << omega << " ";
    cout << "time: " << data.time << " ";
    cout << "drc: " << data.operation.direction << " ";
    //cout << "brake: "<<brake;
    cout << endl;
    count = 0;

    usleep(10000);
	}
	return 0;
}

