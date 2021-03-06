/*
	Motor.h - A library for using the Arduino Due to
	communicate with the ELMO solo gold whistle MC.
	Created by Trent Rankin, trankin8@gatech.edu
	3/30/2018

	Renamed to ELMO and refactored as an arduino library (5/06/2018)
*/

#ifndef ELMOSAM3X_H
#define ELMOSAM3X_H

#include "Arduino.h"
#include <elmo.h>
#include "due_can.h"

//This class is the "library". Different motors can be constructed with differing
//IDs and referenced individually using the different instantiations
class Elmo : public CANListener
{
	public:
		//These functions are gone over in detail in the .cpp file
		Elmo(int id);
		bool connect();
		void disconnect();
		bool isConnected();
		void setId(int);
		int getId();
		bool intSet(char [], int);
		bool floatSet(char [], float, bool);

		bool queryNumbers(char []); // Non-blocking version of get numbers
		bool getNumbers(char [], int*); //Blocking


		bool sendSyncCommand(double*);
		bool motor_on();
		bool set_current(float);
		bool home_position();
		bool get_position(int*); //Blocking
		bool get_velocity(int*);
		bool get_current(float*);
		bool get_aux_position(int*);

		bool query_position(); //Non-Blocking
		bool query_velocity();
		bool query_current();
		bool query_aux_position();

		bool get_reply(float*);
		bool get_reply(int*);
	

		bool send_iq();
		bool get_iq(float*);
	private:
		//The ID parameter is to differentiate different motors
		//The connected paramter is used to tell whether an Elmo has been
		//initialized using a network message (The last command in connect())
		//If the Elmo is not connected, it will not accept PDO messages
		int _id;
		uint8_t _mailboxes[MAILBOXES]; // Vector holding the index of mailboxes assigned to this elmo.
		bool _connected;

		CANRaw &_CANbus; //Reference pointer to the CAN device in use

		// This is a way of monitoring specific frames from a command from TPDO2
		char _activeCommand[2];
		bool _activeQuery;
		uint8_t _queryResponse[4];//buffer that contains the response of the last query

		bool StartMailBoxes();
		void SetFilters(uint16_t baseCOB_ID);
		bool frameHandler(CAN_FRAME &frame, int mailbox);
		void gotFrame(CAN_FRAME *frame, int mailbox);//can_common callback is void

};

#endif

