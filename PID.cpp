#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include "PID.h"
using namespace std;

void init(pid_controller *pid) {
	pid->angleV.theta = pid->angleV.phi = pid->angleV.psi = 0.0;
	pid->prevAngleError.theta = pid->prevAngleError.phi = pid->prevAngleError.psi = 0.0;
	pid->velocity.x = pid->velocity.y = pid->velocity.z = 0.0;
	pid->integral.x = pid->integral.y = pid->integral.z = 0.0;
	pid->prevError.x = pid->prevError.y = pid->prevError.z = 0.0;
	pid->ed = 0;
}

void setPos(pid_controller *pid, float x, float y, float z) {
	pid->pos.x = x;
	pid->pos.y = y;
	pid->pos.z = z;
}

void setParameter(pid_controller *pid, float kp, float ki, float kd) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void setAttitude(pid_controller *pid, float theta, float phi, float psi) {
	pid->attitude.theta = theta;
	pid->attitude.phi = phi;
	pid->attitude.psi = psi;
}

USet updatePos(pid_controller *pid, USet *dst) {
	USet acc, error, derivative;
	Angle angle;
	error.x = dst->x - pid->pos.x;
	error.y = dst->y - pid->pos.y;
	error.z = dst->z - pid->pos.z;
	pid->integral.x += error.x;
	pid->integral.y += error.y;
	pid->integral.z += error.z;
	derivative.x = error.x - pid->prevError.x;
	derivative.y = error.y - pid->prevError.y;
	derivative.z = error.z - pid->prevError.z;
	pid->prevError.x = error.x;
	pid->prevError.y = error.y;
	pid->prevError.z = error.z;
	acc.x = 0.001 * (pid->kp * error.x + pid->ki * pid->integral.x + pid->kd * derivative.x);
	acc.y = 0.001 * (pid->kp * error.y + pid->ki * pid->integral.y + pid->kd * derivative.y);
	acc.z = 0.001 * (pid->kp * error.z + pid->ki * pid->integral.z + pid->kd * derivative.z);
	float maxA = 0.2;
	acc.x = max(min(acc.x, maxA), -maxA);
	acc.y = max(min(acc.y, maxA), -maxA);
	acc.z = max(min(acc.z, maxA), -maxA);
	pid->velocity.x += acc.x;
	pid->velocity.y += acc.y;
	pid->velocity.z += acc.z;
	pid->pos.x += pid->velocity.x;
	pid->pos.y += pid->velocity.y;
	pid->pos.z += pid->velocity.z;
	return acc;
}

void workRotorSpeed(NSet prevRotorSpeed) {
	float n1, n2, n3, n4;
	float F1, F2, F3, F4;
	NSet rotorSpeed;
	F1 = k * prevRotorSpeed.n1 * prevRotorSpeed.n1;
	F2 = k * prevRotorSpeed.n2 * prevRotorSpeed.n2;
	F3 = k * prevRotorSpeed.n3 * prevRotorSpeed.n3;
	F4 = k * prevRotorSpeed.n4 * prevRotorSpeed.n4;
	rotorSpeed.n1 = sqrt((F - 2 * M_THETA) / k4 - M_PHI);
	rotorSpeed.n2 = sqrt((F - 2 * M_THETA) / k4 + M_PHI);
	rotorSpeed.n3 = sqrt((F + 2 * M_THETA) / k4 - M_PHI);
	rotorSpeed.n4 = sqrt((F + 2 * M_THETA) / k4 + M_PHI);
}

Angle workAttitudeAngle(USet acc) {
	float Fsum = sqrt((m * acc.x) * (m * acc.x) +
					  (m * acc.y) * (m * acc.y) +
					  (m * (acc.z + 9.8)) * (m * (acc.z + 9.8)));
	Angle res;
	res.theta = -asin(m * acc.x / Fsum);
	res.phi = asin((m * acc.y) / sqrt(Fsum * Fsum - (m * acc.x) * (m * acc.x)));
	//res.phi = asin((m * acc.y) / (Fsum * cos(res.theta)));
	return res;
}

void updateAttitueAngle(pid_controller *pid, Angle *dst) {
	int count = 0;
	Angle acc, error, derivative;
	error.theta = dst->theta - pid->attitude.theta;
	error.phi = dst->phi - pid->attitude.phi;
	error.psi = dst->psi - pid->attitude.psi;
	derivative.theta = error.theta - pid->prevAngleError.theta;
	derivative.phi = error.phi - pid->prevAngleError.phi;
	derivative.psi = error.psi - pid->prevAngleError.psi;
	pid->prevAngleError.theta = error.theta;
	pid->prevAngleError.phi = error.phi;
	pid->prevAngleError.psi = error.psi;
	acc.theta = 0.001 * (pid->kp * error.theta + pid->kd * derivative.theta);
	acc.phi = 0.001 * (pid->kp * error.phi + pid->kd * derivative.phi);
	acc.psi = 0.001 * (pid->kp * error.phi + pid->kd * derivative.psi);
	float maxA = 0.1;
	acc.theta = max(min(acc.theta, maxA), -maxA);
	acc.phi = max(min(acc.phi, maxA), -maxA);
	acc.psi = max(min(acc.psi, maxA), -maxA);
	pid->angleV.theta += acc.theta;
	pid->angleV.phi += acc.phi;
	pid->angleV.psi += acc.psi;
	pid->attitude.theta += pid->angleV.theta;
	pid->attitude.phi += pid->angleV.phi;
	pid->attitude.psi += pid->angleV.psi;
}

void PIDProcess(pid_controller *pid, USet *dst) {
	USet acc;
	Angle attitude, rst;
	rst.theta = rst.phi = rst.psi = 0.0;
	acc = updatePos(pid, dst);
	attitude = workAttitudeAngle(acc);
	//pid->attitude.theta = attitude.theta;
	//pid->attitude.phi = attitude.phi;
	updateAttitueAngle(pid, &rst);
	cout << pid->pos.x << " " << pid->pos.y << " " << pid->pos.z << " " << 
		    pid->attitude.theta << " " << pid->attitude.phi << " " << pid->attitude.psi << endl;
	pid->queue[pid->ed] = pid->pos;
	pid->queueAngle[pid->ed] = pid->attitude;
	pid->ed ++;
}

bool isOK(pid_controller *pid, USet *dst) {
	if (abs(pid->pos.x - dst->x) < 0.001 && abs(pid->pos.y - dst->y) < 0.001 && abs(pid->pos.z - dst->z) < 0.001
		&& abs(pid->attitude.theta) < 0.001 && abs(pid->attitude.phi) < 0.001 && abs(pid->attitude.psi) < 0.001)
		return true;
	else
		return false;
}

void printTrack(pid_controller *pid) {
	for (int i = 0; i < pid->ed; i++) {
		cout << pid->pos.x << " " << pid->pos.y << " " << pid->pos.z << " " << pid->attitude.theta << " " << pid->attitude.phi << " " << pid->attitude.psi << endl;
	}
}

void fprintTrack(pid_controller *pid) {
	ofstream fout;
	fout.open("output.txt");
	//fout << pid->ed << endl;
	for (int i = 0; i < pid->ed; i++) {
		fout << pid->queue[i].x << " " << pid->queue[i].y << " " << pid->queue[i].z << " " 
			 << pid->queueAngle[i].theta << " " << pid->queueAngle[i].phi << " " << pid->queueAngle[i].psi << endl;
	}
	fout.close();
}