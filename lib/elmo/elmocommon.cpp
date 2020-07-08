#include <elmo.h>

//This is the constructor, pass it the ID of the Elmo MC you want to talk to
//Default ID of Elmos is 127
Elmo::Elmo(int id): _CANbus(ELMO_CANDEV)
{
	this->_id = id;
	this->_connected = false;
	this->_activeQuery =false;
}


bool Elmo::isConnected() {
	return this->_connected;
}

void Elmo::setId(int id) {
	this->_id = id;
}

int Elmo::getId() {
	return this->_id;
}
bool Elmo::motor_on() {
	char command[2];
	int value = 1;
	command[0] = 'M';
	command[1] = 'O';
	return intSet(command, value);
}

//Ser current passes floatset false to speed up execution
bool Elmo::set_current(float value) {
	char command[2];
  
	command[0] = 'T';
	command[1] = 'C';
	return floatSet(command, value, false);
}

bool Elmo::home_position() {
	char command[2];
	int value = 0;
	command[0] = 'P';
	command[1] = 'X';
	return intSet(command, value);
}

bool Elmo::get_position(int* reply) {
	char command[2];
	command[0] = 'P';
	command[1] = 'X';
	return getNumbers(command, reply);
}

bool Elmo::get_velocity(int* reply) {
	char command[2];
	command[0] = 'V';
	command[1] = 'X';
	return getNumbers(command, reply);
}

bool Elmo::get_current(float* reply) {
	char command[2];
	int intVal = 0;
	command[0] = 'T';
	command[1] = 'C';
	typedef union {
		long i;
		float f;
	} fl;
	fl fl1;
	if (getNumbers(command, &intVal)) {
		fl1.i = intVal;
		*reply =  fl1.f;
		return true;
	} else {
		return false;
	}
}

bool Elmo::get_aux_position(int* reply) {
	char command[2];
	command[0] = 'P';
	command[1] = 'Y';
	return getNumbers(command, reply);
}

bool Elmo::query_position() {
	char command[2];
	command[0] = 'P';
	command[1] = 'X';
	return queryNumbers(command);
}

bool Elmo::query_velocity() {
	char command[2];
	command[0] = 'V';
	command[1] = 'X';
	return queryNumbers(command);
}

bool Elmo::query_current() {
	char command[2];
	command[0] = 'T';
	command[1] = 'C';
	return queryNumbers(command);
}

bool Elmo::query_aux_position() {
	char command[2];
	command[0] = 'P';
	command[1] = 'Y';
	return queryNumbers(command);
}

