#ifndef SRILOADCELLSAM3X_H
#define SRILOADCELLSAM3X_H

#include <Arduino.h>
#include <sriloadcell.h>
#include <due_can.h>


class SRILoadcell: public CANListener {
		
	public:
	SRILoadcell(int id);
	bool get_data(LoadcellData_t * reply);
	bool get_rawdata(LoadcellDataRaw_t * reply);
	bool connect();
        bool isConnected();
	void disconnect();

	private:
	
	int _id;
	uint8_t _mailboxes[MAILBOXES]; // Vector holding the index of mailboxes assigned to this elmo.
	bool _connected;

        CANRaw &_CANbus; //Reference pointer to the CAN device in use

	// This is a way of monitoring specific frames from a command from TPDO2
	bool _activeQuery;
	uint8_t _queryResponse[8];//buffer that contains the response of the last query

	bool StartMailBoxes();
	void SetFilters(uint16_t baseCOB_ID);
	bool frameHandler(CAN_FRAME &frame, int mailbox);
	void gotFrame(CAN_FRAME *frame, int mailbox);//can_common callback is
	bool queryData(LoadcellData_t * reply);
	bool queryRawData(LoadcellDataRaw_t * reply);

	int getId();
	void setId(int id);



};

#endif

