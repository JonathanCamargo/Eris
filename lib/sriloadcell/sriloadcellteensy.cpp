#include <Arduino.h>
#if defined(__arm__) && defined(CORE_TEENSY)

#include <sriloadcell.h>
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
bool SRILoadcell::StartMailBoxes(){	
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

void SRILoadcell::SetFilters(uint16_t baseCOB_ID)
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


bool SRILoadcell::frameHandler(CAN_message_t &frame, int mailbox, uint8_t controller){
	//Call back to process messages that arrived to this motor specific mailboxes
	//Serial.print("isr:");Serial.print("MB[");Serial.print(mailbox);
	//Serial.print("]=");
	//printFrame(frame);

	//If there is a pending query from queryNumbers lets check if this frame
	// belongs to it and then save the data. Use activeQuery and activeCommand members to check for  
	
	if (this->_activeQuery){
			//Copy the response and disable the active query		
			this->_activeQuery=false;
			memcpy(this->_queryResponse,frame.buf,8);
			return true; //Don't let FlexCAN handle this frame
	}
	else{// Any other message just let FlexCAN handle it
		return false;
	}
}

//Use this function before trying to send PDOs to an SRILoadcell,
//a disconnected SRILoadcell will not accept or reply to PDOs
//This is a long function that takes a lot of research to understand
//It should be used only one time per SRILoadcell, if a disconnect is not sent
//between connect messages the SRILoadcell will have to be restarted
bool SRILoadcell::connect() {

	SRILoadcell::StartMailBoxes();
	//Serial.println(this->_id);
	//Serial.println(this->_mailboxes[0]);
	//Serial.println(this->_mailboxes[1]);
	this->SetFilters(0x000);
	this->_connected = true;

	//Attach this object for callbacks	
	this->_CANbus.attachObj(this);
	for(int i=0;i<MAILBOXES;i++){
		this->attachMBHandler(this->_mailboxes[i]);
	}
	return true;
}

//This function is used to put the SRILoadcell back into pre-operational state
//Depending on the number in byte 0 different functions can be performed
//Consult the NMT section in the CANopen guide for other functions
void SRILoadcell::disconnect() {
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


//This function will query values from the SRILoadcell using RPDO2 (binary interpreter)
//This function returns an int, but floats can be queried as well
//This will have to be handled by the calling function, see the typedef above
bool SRILoadcell::queryData(LoadcellData_t * reply) {
    CAN_message_t message;
    message.id = 0x000 + this->_id;
    message.flags.extended = false;
    message.flags.remote = 0;
    message.len = 8;
    // Send 0x00 for all (SRILoadcell is dummy and does not care)
    message.buf[0] = 0x00;
    message.buf[1] = 0x00;
    message.buf[2] = 0x00;
    message.buf[3] = 0x40;
    message.buf[4] = 0x00;
    message.buf[5] = 0x00;
    message.buf[6] = 0x00;
    message.buf[7] = 0x00;

// Inform that we want to query something so that the interrupt knows what to look for.
    this->_activeQuery=true;
    if (this->_CANbus.write(message)) {
		delayMicroseconds(500);
		if (!this->_activeQuery){
				//Serial.println("funciono");
				//Take the int from the response buffer
				LoadcellData_t data=frame2data(this->_queryResponse);
				memcpy(reply,&data,sizeof(LoadcellData_t));			
				return true;
		}else {
			return false;
		}
    }else {
		//Serial.print("\nERROR!");
		return false;
    }
}

bool SRILoadcell::queryRawData(LoadcellDataRaw_t * reply) {
   
   CAN_message_t message;
    message.id = 0x000 + this->_id;
    message.flags.extended = false;
    message.flags.remote = 0;
    message.len = 8;
    // Send 0x00 for all (SRILoadcell is dummy and does not care)
    message.buf[0] = 0x00;
    message.buf[1] = 0x00;
    message.buf[2] = 0x00;
    message.buf[3] = 0x40;
    message.buf[4] = 0x00;
    message.buf[5] = 0x00;
    message.buf[6] = 0x00;
    message.buf[7] = 0x00;
// Inform that we want to query something so that the interrupt knows what to look for.
    this->_activeQuery=true;
    if (this->_CANbus.write(message)) {
		delayMicroseconds(5000);
		if (!this->_activeQuery){
				//Serial.println("funciono");
				//Take the int from the response buffer
				memcpy(reply,this->_queryResponse,sizeof(LoadcellDataRaw_t));			
				return true;
		}else {
			return false;
		}
    }else {
		//Serial.print("\nERROR!");
		return false;
    }
}

#endif


