#include <Arduino.h>
#if defined(__arm__) && defined(CORE_TEENSY)

#include <elmo.h>
#include <FlexCAN.h>

static void printFrame(CAN_message_t &frame){
   Serial.print("ID: ");
   Serial.print(frame.id, HEX);
   Serial.print(" Data: ");
   for (int c = 0; c < frame.len; c++) 
   {
      Serial.print(frame.buf[c], HEX);
      Serial.write(' ');
   }
   Serial.write('\r');
   Serial.write('\n');
}


//Setup filters
// I will use more than one MAILBOX to handle messages from this device.
// to avoid conflicting with mailboxes used by others, this class checks 
// for active filter in all the mailboxes and picks the ones that are available.
bool Elmo::StartMailBoxes(){	
	uint8_t mbtaken=0;

	CAN_filter_t filter;
	filter.flags.extended=0;
	filter.flags.remote=0;
	
	for (int i=0;i<this->_CANbus.getNumRxBoxes();i++){
		if (this->_CANbus.getFilter(filter,i)&&filter.id==0){ 
		//Mailbox is valid and filter is not in use	
		//Fill this with the filter and save the index of the mailbox for further use.
		if (mbtaken<MAILBOXES){
			filter.id=this->_id;
			this->_CANbus.setFilter(filter,i);
			this->_CANbus.setMask(0x00FF0000,i);	
			this->_mailboxes[mbtaken]=i;
			mbtaken++;
		}
		else{ //Set filters for the rest of the mailboxes
			filter.id=0;			
			this->_CANbus.setFilter(filter,i);
			this->_CANbus.setMask(0x00FF0000,i);	
		}
		this->_CANbus.getFilter(filter,i);
	}
	}
	if (mbtaken<MAILBOXES){
		//FATAL ERROR
		return false;	
	}
	else{
		return true;	
	}
}

void Elmo::SetFilters(uint16_t baseCOB_ID)
{
	//Set up the mailbox filters to only get an specific baseCOB_ID
	CAN_filter_t filter;
	filter.id=baseCOB_ID+this->_id;
	filter.flags.extended=0;
	filter.flags.remote=0;
	for (int i=0;i<MAILBOXES;i++){
		this->_CANbus.setFilter(filter,this->_mailboxes[i]);
		this->_CANbus.setMask(0x0FFF0000,this->_mailboxes[i]);		
	}
}


bool Elmo::frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller){
	//Call back to process messages that arrived to this motor specific mailboxes
	//Serial.print("isr:");Serial.print("MB[");Serial.print(mailbox);
	//Serial.print("]=");
	//printFrame(frame);

	//If there is a pending query from queryNumbers lets check if this frame
	// belongs to it and then save the data. Use activeQuery and activeCommand members to check for  
	
	if (this->_activeQuery){
		if (this->_activeCommand[0]==frame.buf[0] && this->_activeCommand[1]==frame.buf[1]){
			//Copy the response and disable the active query		
			this->_activeQuery=false;
			memcpy(this->_queryResponse,&frame.buf[4],4);
			return true; //Don't let FlexCAN handle this frame
		} 
		return false;
	}
	else{// Any other message just let FlexCAN handle it
		return false;
	}
}

//Use this function before trying to send PDOs to an Elmo,
//a disconnected Elmo will not accept or reply to PDOs
//This is a long function that takes a lot of research to understand
//It should be used only one time per Elmo, if a disconnect is not sent
//between connect messages the Elmo will have to be restarted
bool Elmo::connect() {

	Elmo::StartMailBoxes();
	this->SetFilters(0x580);

	//Must declare a CAN_message_t to set up the message, see the
	//due_can documentation for details
	CAN_message_t message;
	//0x600 is the base COB-ID of SDO messages, the specific
	//Elmo ID must be added to this or the Elmo will not accept the frame
    	message.id = 0x600 + this->_id;
	//This message only needs an 11-bit identifier
    	message.flags.extended = false;
    	message.flags.remote = 0;
    	message.len = 8;
	
	//This large section of messages is a sequence that sets up a mapped PDO
	//for PU and VU to make the request faster (it does not have to go through
	//the binary interpreter (if this is confusing, consult the CANopen guide)
	
	//This message disables TPDO3 (the PDO to be mapped)
	//This is necessary to change the parameters of TPDO3
    	message.buf[0] = 0x2f;
    	message.buf[1] = 0x02;
    	message.buf[2] = 0x18;
    	message.buf[3] = 0x01;
    	message.buf[4] = 0xff;
    	message.buf[5] = 0x03;
    	message.buf[6] = 0x00;
    	message.buf[7] = 0x80;
    	this->_CANbus.write(message);
    	delay(10);

	//This message sets the number of mapped objects in TPDO3 to 0, necessary before changing mapping
	message.buf[0] = 0x2f;
	message.buf[1] = 0x02;
	message.buf[2] = 0x1a;
	message.buf[3] = 0x00;
	message.buf[4] = 0x00;
	message.buf[5] = 0x00;
	message.buf[6] = 0x00;
	message.buf[7] = 0x00;
	this->_CANbus.write(message);
	delay(10);

	//This message maps PU to the first 4 bytes of TPDO3
	message.buf[0] = 0x2f;
	message.buf[1] = 0x02;
	message.buf[2] = 0x1a;
	message.buf[3] = 0x01;
	message.buf[4] = 0x20;
	message.buf[5] = 0x00;
	message.buf[6] = 0x63;
	message.buf[7] = 0x60;
	this->_CANbus.write(message);
	delay(10);

	//This message maps VU as the second 4 bytes of TPDO3
	message.buf[0] = 0x2f;
	message.buf[1] = 0x02;
	message.buf[2] = 0x1a;
	message.buf[3] = 0x02;
	message.buf[4] = 0x20;
	message.buf[5] = 0x00;
	message.buf[6] = 0x6c;
	message.buf[7] = 0x60;
	this->_CANbus.write(message);
	delay(10);

	//This message sets TPDO3 to be sent every SYNC message
	message.buf[0] = 0x2f;
	message.buf[1] = 0x02;
	message.buf[2] = 0x18;
	message.buf[3] = 0x02;
	message.buf[4] = 0x01;
	message.buf[5] = 0x00;
	message.buf[6] = 0x00;
	message.buf[7] = 0x00;
	this->_CANbus.write(message);
	delay(10);

	//This message sets the number of mapped objects in TPDO3 to 2 (PU and VU)
	message.buf[0] = 0x2f;
	message.buf[1] = 0x02;
	message.buf[2] = 0x1a;
	message.buf[3] = 0x00;
	message.buf[4] = 0x02;
	message.buf[5] = 0x00;
	message.buf[6] = 0x00;
	message.buf[7] = 0x00;
	this->_CANbus.write(message);
	delay(10);

	//This message enables TPDO3
	message.buf[0] = 0x2f;
	message.buf[1] = 0x02;
	message.buf[2] = 0x18;
	message.buf[3] = 0x01;
	message.buf[4] = 0xff;
	message.buf[5] = 0x03;
	message.buf[6] = 0x00;
	message.buf[7] = 0x00;
	this->_CANbus.write(message);
	delay(10);
	
	//This section is for sending the start-up NMT message to put the servo in the operational state
	//A servo will only respond to SDOs if it is not in the operational state
	//The id of NMT messages is all zeros
	message.id = 0x000;
	message.len = 2;
	message.buf[0] = 0x01;
	message.buf[1] = this->_id;
	this->_CANbus.write(message);
	delay(10);
	//This command sets the receive filter to only receive SDOs from the Elmo with
	//the ID that the messages are being sent to
	//Trent used filter with 0x580 base to only get SDO messages
	//but configuring the filters after writing the message and receiving a reply does not make sence that should go before any write.	

	if (this->_CANbus.available()) {
		//Set up a frame to receive the reply data
		CAN_message_t incoming;
		//Read all of the buffered CAN messages,
		//in this case there will be a reply from every SDO sent above
		while (this->_CANbus.available()) {
			this->_CANbus.read(incoming);
			printFrame(incoming);
		}
		this->_connected = true;
	} else {
		return false;
	}

	// Change filters to catch TPDO2 messages only and attach the interrupt to process them.
	this->SetFilters(0x280);
	//Attach this object for callbacks	
	this->_CANbus.attachObj(this);
	for(int i=0;i<MAILBOXES;i++){
		this->attachMBHandler(this->_mailboxes[i]);
	}
	return true;
}

//This function is used to put the Elmo back into pre-operational state
//Depending on the number in byte 0 different functions can be performed
//Consult the NMT section in the CANopen guide for other functions
void Elmo::disconnect() {
	CAN_message_t message;
	message.id = 0x000;
	message.flags.extended = false;
	message.flags.remote = 0;
	message.len = 2;
	message.buf[0] = 0x02;
	message.buf[1] = this->_id;
	this->SetFilters(0x580);
	if (this->_CANbus.write(message)) {
		delay(10);
		if (this->_CANbus.available()) {
			CAN_message_t incoming;
			while (this->_CANbus.available()) {
				this->_CANbus.read(incoming);
			}
			this->_connected = false;
		} else {
			//Serial.print("\nNo message 2");
		}
	}else {
	}
}

//This function will set an integer value using the binary interpreter in RPDO2
bool Elmo::intSet(char command[], int value) {
	CAN_message_t message;
	//TPDO2 has a base ID of 0x300
	message.id = 0x300 + this->_id;
	message.flags.extended = false;
	message.flags.remote = 0;
	message.len = 8;
	//Bytes 0 and 1 are the ascii command characters
	message.buf[0] = command[0];
	message.buf[1] = command[1];
	message.buf[2] = 0x00;
	message.buf[3] = 0x00;
	//Bytes 4-7 are the integer value to set
	memcpy(&message.buf[4],&value,sizeof(value));

	//The reply will be sent out of TPDO2 which has a base id of 0x280
	//CAN_filter_t filter;
	//filter.id=0x280 + this->_id;
	//filter.flags.extended=0;
	//filter.flags.remote=0;
	//this->_CANbus.setFilter(filter, 0x280 + this->_id);
	if (this->_CANbus.write(message)) {
		delay(10);
		if (this->_CANbus.available()) {
			CAN_message_t incoming;
			while (this->_CANbus.available()) {
				this->_CANbus.read(incoming);
			}
			//The return is based on whether the reply ever came from the Elmo
			return true;
		} else {
			//Serial.print("\nNo message 3");
			return false;
		}
	}else {
		return false;
	}
}

//This function is very similar to intset, but it can set a float
bool Elmo::floatSet(char command[], float value, bool wait) {
	CAN_message_t message;
	message.id = 0x300 + this->_id;
	message.flags.extended = false;
	message.flags.remote = 0;
	message.len = 8;

	//This typedef is used because if the float was stored as an int
	//the bits would change and the wrong thing would be sent
	//Instead, f.i will preserve the bits but allow the value to be
	//treated as an integer
	typedef union {
		long i;
		float f;
	} fl;
	fl fl1;
	fl1.f = value;

	message.buf[0] = command[0];
	message.buf[1] = command[1];
	message.buf[2] = 0x00;
	message.buf[3] = 0x80;
	memcpy(&message.buf[4],&fl1.i,sizeof(fl1.i));

	//CAN_filter_t filter;
	//filter.id=0x280 + this->_id;
	//filter.flags.extended=0;
	//filter.flags.remote=0;
	//this->_CANbus.setFilter(filter, 0x280 + this->_id);	//This conditional is so that a command can be sped up by not waiting for a reply
	//Intended to make setting TC faster
// Inform that we want to query something so that the interrupt knows what to look for.
	if(wait) {
		if (this->_CANbus.write(message)) {
			delay(5);
			
			if (this->_CANbus.available()) {
				CAN_message_t incoming;
				while (this->_CANbus.available()) {
					this->_CANbus.read(incoming);
					//printFrame(incoming);
				}
				return true;
			} else {
				//Serial.print("\nNo message 4");
				return false;
			}
			
		}else {
			return false;
		}
	} else {
		this->_CANbus.write(message);
		return true;
	}
	
}



//This function will query values from the elmo using RPDO2 (binary interpreter)
//This function returns an int, but floats can be queried as well
//This will have to be handled by the calling function, see the typedef above
bool Elmo::getNumbers(char command[], int* reply) {
    if (!queryNumbers(command)){
	return false;	
	}
   delayMicroseconds(1500);
   if (!this->_activeQuery){
	//Take the int from the response buffer
	memcpy(reply,this->_queryResponse,sizeof(int));			
	return true;
   }else {
 	return false;
   }
}

bool Elmo::get_reply(float* reply){
	 if (!this->_activeQuery){
		memcpy(reply,this->_queryResponse,sizeof(float));
		return true;	
	}else{
		return false;		
	}
}

bool Elmo::get_reply(int* reply){
	 if (!this->_activeQuery){
		memcpy(reply,this->_queryResponse,sizeof(int));
		return true;	
	}else{
		return false;		
	}
}

bool Elmo::queryNumbers(char command[]) {
    CAN_message_t message;
    message.id = 0x300 + this->_id;
    message.flags.extended = false;
    message.flags.remote = 0;
    message.len = 8;
    //The first two bytes are the ascii command characters
    message.buf[0] = command[0];
    message.buf[1] = command[1];
    message.buf[2] = 0x00;
    message.buf[3] = 0x40;
    message.buf[4] = 0x00;
    message.buf[5] = 0x00;
    message.buf[6] = 0x00;
    message.buf[7] = 0x00;


// Inform that we want to query something so that the interrupt knows what to look for.
	//this->InformQuery(command);
    this->_activeCommand[0]=command[0];
    this->_activeCommand[1]=command[1];			
    this->_activeQuery=true;

    if (this->_CANbus.write(message)) {
		return true;
    }else {
		//Serial.print("\nERROR!");
		return false;
    }
}

//This function will send a SYNC command to all devices on the bus
//The SYNC command was set up in the connect function to reply with
//PU and VU, but this can be changed
bool Elmo::sendSyncCommand(double* reply) {
	CAN_message_t message;
	message.id = 0x080;
	message.flags.extended = false;
	message.flags.remote = 0;
	message.len = 8;
	message.buf[0] = 0;
	message.buf[1] = 0;
	message.buf[2] = 0;
	message.buf[3] = 0;
	message.buf[4] = 0;
	message.buf[5] = 0;
	message.buf[6] = 0;
	message.buf[7] = 0;

	//The reply will come as TPDO3, as set in connect()
	//CAN_filter_t filter;
	//filter.id=0x380 + this->_id;
	//filter.flags.extended=0;
	//filter.flags.remote=0;
        //this->_CANbus.setFilter(filter, 0x380 + this->_id);
	if (this->_CANbus.write(message)) {
		delay(5);
		if (this->_CANbus.available()) {
			CAN_message_t incoming;
			while (this->_CANbus.available()) {
				this->_CANbus.read(incoming);
				memcpy(reply,incoming.buf,64);
				//*reply = incoming.data.value;
			}
			return true;
		}else {
			Serial.print("\nNo message 6");
			return false;
		}
	}else {
		return false;
	}
}

//The next two functions are special to using CAN in our RTOS program
//Waiting for a reply takes too long, so it is split up into two functions
//This allows the RTOS to delay the task instead of waiting
bool Elmo::send_iq() {
	CAN_message_t message;
	message.id = 0x300 + this->_id;
    message.flags.extended = false;
    message.flags.remote = 0;
    message.len = 8;
	
	//The first two bytes are the ascii command characters
	message.buf[0] = 'I';
    message.buf[1] = 'Q';
    message.buf[2] = 0x00;
    message.buf[3] = 0x40;
    message.buf[4] = 0x00;
    message.buf[5] = 0x00;
    message.buf[6] = 0x00;
    message.buf[7] = 0x00;
	
	//Send the message and return, do not wait for a reply
        //CAN_filter_t filter;
	//filter.id=0x280 + this->_id;
	//filter.flags.extended=0;
	//filter.flags.remote=0;
        //this->_CANbus.setFilter(filter, 0x280 + this->_id);
	if(this->_CANbus.write(message)) {
		return true;
	} else {
		return false;
	}
}

//This function should only be called ~5 milliseconds after send_iq was called
bool Elmo::get_iq(float *reply) {
	/*typedef union {
		long i;
		float f;
	} fl;
	fl fl1;*/

	if (this->_CANbus.available()) {
		CAN_message_t incoming;
		while (this->_CANbus.available()) {
			this->_CANbus.read(incoming);
			//Bytes 4-7 contain the queried value, make sure that the reply is IQ
			if (incoming.buf[0] == 'I') {
				memcpy(reply,&incoming.buf[4],4);
				//fl1.i = incoming.data.high;
				//*reply =  fl1.f;
			}
		}
		return true;
	}else {
		//Serial.print("\nNo message 7");
		return false;
	}
}

#endif


