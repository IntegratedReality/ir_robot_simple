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
    // ここを弄るといいらしい

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

    // オーバードライブ防止処理
    //bool F, B;
    //checkMovable(data.pos, F, B);
    //前に動く許可がF, 後ろに動く許可がBに代入される

    //bool MoveSURUNO = true;

    double ofsetRotate = 2.7;
    double ofsetMoving = 1.0;
    bool brake{false};

    //動作許可を反映してトルク設定
    if (permissions[static_cast<int>(data.operation.direction)]){
      switch (data.operation.direction){
        case NO_INPUT:
          brake = true;
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
      brake = true;
      //許可がない場合.
      //vやomegaはスコープ内で宣言してるから0になってくれるので放置でOK
    }
    /*
    switch ((ID < 3 && !data.isAI)
        ? data.operation.direction :
        ai.getOperation().direction
        ) {
      case NO_INPUT:
        break;
      case TOP:
        v = (double)F * max_v;
        MoveSURUNO = F;
        break;
      case TOP_RIGHT:
        v = (double)F * max_v;
        omega = -ofsetMoving * max_omega;
        MoveSURUNO = F;
        break;
      case RIGHT:
        omega = - ofsetRotate * max_omega;
        break;
      case BOTTOM_RIGHT:
        v = -((double)B * max_v);
        omega = ofsetMoving * max_omega;
        MoveSURUNO = B;
        break;
      case BOTTOM:
        v = -((double)B * max_v);
        MoveSURUNO = B;
        break;
      case BOTTOM_LEFT:
        v = -((double)B * max_v);
        omega = -ofsetMoving * max_omega;
        MoveSURUNO = B;
        break;
      case LEFT:
        omega = ofsetRotate * max_omega;
        break;
      case TOP_LEFT:
        v = (double)F * max_v;
        omega = ofsetMoving * max_omega;
        MoveSURUNO = F;
        break;
    }

    bool active = true;
    if (data.state == DEAD || data.state == STANDBY) active = false;
    */

    mutex_obj.lock();
    drive.setTarget
      (v,
       omega,
       (brake ? MotorMode::Brake : MotorMode::Move)
       );
    mutex_obj.unlock();
    // ここまでを弄る

    if (count != 3000) {
      count++;
      continue;
    }
    //	cout << F << B << endl;
    cout << "v: " << v << " ";
    cout << "omega: " << omega << " ";
    cout << "x: " << data.pos.x << " ";
    cout << "y: " << data.pos.y << " ";
    cout << "theta: " << data.pos.theta << " ";
    cout << "time: " << data.time << " ";
    //cout << "isAI: " << data.isAI << " ";
    //cout << "state: " << data.state << " ";
    cout << "drc: " << data.operation.direction << " ";
    //cout << "shot: " << ((ID < 3 && !data.isAI) ? data.operation.shot : ai.getOperation().shot) << " ";
    cout << endl;
    count = 0;

    usleep(10000);
	}
	return 0;
}

//Position p1(1800 / 4., 2700 / 4., 0);
//Position p2(2 * 1800 / 4., 2 * 2700 / 4., 0);
//Position p3(3 * 1800 / 4., 3 * 2700 / 4., 0);

/*
void checkMovable(Position _pos, bool &_F, bool &_B) {
	double vx = cos(_pos.theta), vy = sin(_pos.theta);
	_F = true;
	_B = true;

  Eigen::Vector2f velocity(vx, vy);

	// フィールド上辺
	if (_pos.y < 130) {
		if (vy < 0) _F = false; 
		else _B = false;
	} 
	// 右辺
	if (_pos.x > 1800 - 130) {
		if (vx > 0) _F = false; 
		else _B = false;
	}
	// 下辺
	if (_pos.y > 2700 - 130) {
		if (vy > 0) _F = false; 
		else _B = false;
	}
	// 左辺
	if (_pos.x < 130) {
		if (vx < 0) _F = false; 
		else _B = false;
	}
	// ポイントオブジェクト1
	if (distance(_pos, p1) < 125 + 100 + 30) {
    Eigen::Vector2f dirToObj(p1.x-_pos.x, p1.y-_pos.y);
		if (dirToObj.dot(velocity) > 0) _F = false;
		else _B = false;
	}
	// ポイントオブジェクト2
	if (distance(_pos, p2) < 125 + 100 + 30) {
    Eigen::Vector2f dirToObj(p2.x-_pos.x, p2.y-_pos.y);
		if (dirToObj.dot(velocity) > 0) _F = false;
		else _B = false;
	}
	// ポイントオブジェクト3
	if (distance(_pos, p3) < 125 + 100 + 30) {
    Eigen::Vector2f dirToObj(p3.x-_pos.x, p3.y-_pos.y);
		if (dirToObj.dot(velocity) > 0) _F = false;
		else _B = false;
	}
}

double distance(Position _p1, Position _p2) {
	return sqrt(pow(_p1.x - _p2.x, 2.) + pow(_p1.y - _p2.y, 2.));
}

*/
