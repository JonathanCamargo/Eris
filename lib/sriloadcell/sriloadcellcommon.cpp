#include <sriloadcell.h>

int bit2sign(bool bit){
  int sign=1;
  if(bit!=0){
    sign=-1;
  }
  return sign;
}

LoadcellData_t frame2data(uint8_t *frameDataBytes){

      //From a can frame data (bindata) retrieve tha loadcell data in a struct
      LoadcellData_t returnData;
      
      int forceXSign = bit2sign(frameDataBytes[0] & 0x80);
      int forceYSign = bit2sign((frameDataBytes[1]/16) % 2);
      int forceZSign = bit2sign((frameDataBytes[2]%4) > 1);
      int momentXSign = bit2sign(((frameDataBytes[4]/16) % 4) > 1);
      int momentYSign = bit2sign(((frameDataBytes[5]%16) > 7));
      int momentZSign = bit2sign(frameDataBytes[6] & 0x02);

      int forceX_int = ((frameDataBytes[0]%128) << 3) | frameDataBytes[1]/32;
      float _forceX = (float)forceX_int * forceXSign;

      int forceY_int = ((frameDataBytes[1]%16) << 6) | frameDataBytes[2]/4;
      float _forceY = (float)forceY_int * forceYSign;

      int forceZ_int = ((((frameDataBytes[2] % 2) << 8) | frameDataBytes[3]) << 2) | frameDataBytes[4]/64;
      float _forceZ = (float)forceZ_int * forceZSign;

      int momentX_int = ((frameDataBytes[4]%32) << 4) | frameDataBytes[5]/16;
      float _momentX = (float)momentX_int * momentXSign / MOMENT_SCALING_FACTOR;

      int momentY_int = ((frameDataBytes[5]%8) << 6) | frameDataBytes[6]/4;
      float _momentY = (float)momentY_int * momentYSign / MOMENT_SCALING_FACTOR;

      int momentZ_int = (((frameDataBytes[6] % 2) << 8) | frameDataBytes[7]);
      float _momentZ = (float)momentZ_int * momentZSign / MOMENT_SCALING_FACTOR;

	returnData.forceX = _forceX;
	returnData.forceY = -_forceY;
	returnData.forceZ = -_forceZ;
	returnData.momentX  = _momentX;
	returnData.momentY = -_momentY;
	returnData.momentZ = -_momentZ;

	return returnData;
}


//This is the constructor, pass it the ID of the SRILoadcell
SRILoadcell::SRILoadcell(int id): _CANbus(CANDEV)
{
	this->_id = id;
	this->_connected = false;
	this->_activeQuery =false;
	
}

bool SRILoadcell::isConnected() {
	return this->_connected;
}

void SRILoadcell::setId(int id) {
	this->_id = id;
}

int SRILoadcell::getId() {
	return this->_id;
}


bool SRILoadcell::get_data(LoadcellData_t * reply){
	return this->queryData(reply);
}


bool SRILoadcell::get_rawdata(LoadcellDataRaw_t * reply){
	return this->queryRawData(reply);
}
