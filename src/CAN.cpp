#include <CAN.h>
#include <FlexCAN.h>

bool CanControl::_can0 = false;
bool CanControl::_can1 = false;
uint8_t CanControl::canbus = 0;
std::map<uint32_t ,CAN_message_t> buffer_table;
std::vector<CAN_message_t> msg_stack;


CanControl* CanControl::CreateCanControl(uint8_t _canbus){
	if(_canbus == 0){
		static CanControl CAN0(0);
		canbus = 0;
		return &CAN0;
	}else if(_canbus == 1){
		static CanControl CAN1(1);
		canbus = 1;
		return &CAN1;
	}else{
		return NULL;
	}
}

CanControl::CanControl(uint8_t CANBus){
	if(CANBus == 0){
		CAN = &Can0;
	}else
	if(CANBus == 1){
		CAN = &Can1;
	}
	is_can_open = is_start_Com = I_have_buffer_data = false;
}

void CanControl::init(int baudrate){
	CAN->begin(baudrate);
	is_can_open = true;
}

void CanControl::resetTable(){
	buffer_table.clear();
	I_have_buffer_data = false;
}

void CanControl::CANDataPull(uint32_t id ,uint8_t data[8]){
	CAN_message_t msg = buffer_table[id];
	memcpy(data,msg.buf,8);
}

int8_t CanControl::CANAllDataRead(){
	if(!is_can_open)return -1;
	static uint8_t data_vanishing;
	CAN_message_t msg;
	data_vanishing++;
	while(CAN->available()){
		I_have_buffer_data = true;
		CAN->read(msg);
		buffer_table[msg.id] = msg;
		data_vanishing = 0;
	}
		is_start_Com = true;
	if(data_vanishing == 0){
		return 0;
	}else if(data_vanishing < 16){
		return data_vanishing;
	}else {
		is_start_Com = false;
		return -1;
	}
	
}

	CAN_message_t buff_msg;
void CanControl::CANDataPush(uint32_t id ,uint8_t data[8]){
	MSG.id = id;
	memcpy(MSG.buf,data,8);

	msg_stack.push_back(MSG);
	return;
}

void CanControl::MsgStackClear(){
	msg_stack.clear();
}

CAN_message_t sendMSG;
void CanControl::CANMsgWrite(CAN_message_t msg){
	if(!is_can_open)return;
	CAN->write(msg);
	return;
}
int8_t CanControl::CANAllDataWrite(){
	if(!is_can_open)return -1;
	int8_t s = 0;
	for(CAN_message_t buff : msg_stack){
		CAN->write(buff);
		s++;
	}
	msg_stack.clear();
	return s;
}