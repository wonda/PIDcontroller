#include "PID.h"
#include <cstdlib>
#include <iostream>
#include <fstream>
using namespace std;

int main() {
	pid_controller pid;
	init(&pid);
	setParameter(&pid, 2.379, 0.0, 90.0);
	setPos(&pid, 0.0, 0.0, 40.0);
	setAttitude(&pid, 1.0, 1.0, 1.0);
	USet dst;
	dst.x = 60.0; dst.y = 80.0; dst.z = 100.0;
	while (!isOK(&pid, &dst)) {
		PIDProcess(&pid, &dst);
	}
	cout << pid.ed << endl;
	fprintTrack(&pid);
	/*//find out best kp
	ofstream fout;
	fout.open("kd.txt");
	float MINKD = 3.0, kd = 30;
	int count = 0, MIN = 20000;
	while (kd < 200) {
		kd ++;
		pid_init(&pid);
		pid_set_parameter(&pid, 3.0, 0.0, kd);
		pid_set_pos(&pid, 0.0, 0.0, 100.0);
		updatePos(&pid, &dst);
		if (pid.ed < MIN) {
			MIN = pid.ed;
			MINKD = kd;
		}
		fout << kd << " " << pid.ed << endl;
		count ++;
	}
	cout << MINKD << " " << MIN << endl;
	fout.close();*/
	system("pause");
	return 0;
}