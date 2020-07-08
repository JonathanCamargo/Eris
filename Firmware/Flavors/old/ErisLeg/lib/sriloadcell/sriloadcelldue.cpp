#include <Arduino.h>
#if defined(__SAM3X8E__) || defined(__SAM3X8H__)

#include <sriloadcell.h>
#include <due_can.h>

static void printFrame(CAN_FRAME &frame){
   Serial.print("ID: ");
   Serial.print(frame.id, HEX);
   Serial.print(" Data: ");
   for (int c = 0; c < frame.length; c++) 
   {
      Serial.print(frame.data.bytes[c], HEX);
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

	for (int i=0;i<MAILBOXES;i++){
		int mb=this->_CANbus.findFreeRXMailbox();
		if (mb==-1){
		//FATAL ERROR
		return false;
		}
		this->_mailboxes[i]=mb;		
		this->_CANbus.setRXFilter(this->_mailboxes[i],this->_id,0x0FF,false);
	}
	return true;
}

void SRILoadcell::SetFilters(uint16_t baseCOB_ID)
{
	for (int i=0;i<MAILBOXES;i++){
		//Serial.print("setting filter for mb:(");
		//Serial.print(this->_mailboxes[i]);
		//Serial.print(") ");
		//Serial.println(this->_id);
		//Serial.print(" mask:");
		//Serial.println(0x0FFF0000);
		this->_CANbus.setRXFilter(this->_mailboxes[i],baseCOB_ID+this->_id,0xFFF,false);
	}
}

void SRILoadcell::gotFrame(CAN_FRAME *frame,int mailbox){
	frameHandler(*frame,mailbox);
}





bool SRILoadcell::frameHandler(CAN_FRAME &frame,int mailbox){
	//Call back to process messages that arrived to this motor specific mailboxes
	//Serial.print("isr:");Serial.print("MB[");Serial.print(mailbox);
	//Serial.print("]=");
	//Serial.print("mb(");Serial.print(mailbox);Serial.print(")");printFrame(frame);

	//If there is a pending query from queryData lets check if this frame
	// belongs to it and then save the data. Use activeQuery and activeCommand members to check for  
	
	if (this->_activeQuery){		
			this->_activeQuery=false;
			memcpy(this->_queryResponse,frame.data.bytes,8);
			return true; //Don't let FlexCAN handle this frame
	}
	else{ 
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
    this->SetFilters(0x000);  
 
    CAN_FRAME message;
	
    message.id = 0x000;
    message.extended = false;
    message.rtr = 0;
    message.length = 2;
	message.data.bytes[0] = 0x02;
    message.data.bytes[1] = this->_id;
	
	
    if (this->_CANbus.sendFrame(message)) {
		delayMicroseconds(10);
		if (this->_CANbus.rx_avail()) {
			CAN_FRAME incoming;
			//this->_CANbus.mailbox_read(_MAILBOX, &incoming);
			this->_connected = false;
		} else {
			Serial.print("\nNo message 2");
		}
    }else {
		//Serial.print("\nERROR!");
    }
}


//This function will query values from the SRILoadcell 
bool SRILoadcell::queryData(LoadcellData_t * reply) {
   
    CAN_FRAME message;
    message.id = 0x000 + this->_id;
    message.extended = false;
    message.rtr = 0;
    message.length = 8;
    // Send 0x00 for all (SRILoadcell is dummy and does not care)
    message.data.bytes[0] = 0x00;
    message.data.bytes[1] = 0x00;
    message.data.bytes[2] = 0x00;
    message.data.bytes[3] = 0x00;
    message.data.bytes[4] = 0x00;
    message.data.bytes[5] = 0x00;
    message.data.bytes[6] = 0x00;
    message.data.bytes[7] = 0x00;

// Inform that we want to query something so that the interrupt knows what to look for.
    this->_activeQuery=true;
    if (this->_CANbus.sendFrame(message)) {
		delayMicroseconds(5000);
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
   
    CAN_FRAME message;
    message.id = 0x000 + this->_id;
    message.extended = false;
    message.rtr = 0;
    message.length = 8;
    // Send 0x00 for all (SRILoadcell is dummy and does not care)
    message.data.bytes[0] = 0x00;
    message.data.bytes[1] = 0x00;
    message.data.bytes[2] = 0x00;
    message.data.bytes[3] = 0x00;
    message.data.bytes[4] = 0x00;
    message.data.bytes[5] = 0x00;
    message.data.bytes[6] = 0x00;
    message.data.bytes[7] = 0x00;

// Inform that we want to query something so that the interrupt knows what to look for.
    this->_activeQuery=true;
    if (this->_CANbus.sendFrame(message)) {
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
