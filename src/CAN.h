#ifndef CAN_H
#define CAN_H

#include <vector>
#include <map>
#include <FlexCAN.h>

class CanControl{
	
	public:
	
		static CanControl *CreateCanControl(uint8_t _canbus);
	
		bool is_can_open;
		bool is_start_Com;
		bool I_have_buffer_data;
		
		void init(int baudrate);
		void resetTable();
		void CANDataPull(uint32_t id ,uint8_t data[8]);
		int8_t CANAllDataRead();
		void CANDataPush(uint32_t id ,uint8_t data[8]);
		void MsgStackClear();
		void CANMsgWrite(CAN_message_t msg);
		int8_t CANAllDataWrite();

	private:
		FlexCAN *CAN;
		CAN_message_t MSG;
		
		static uint8_t canbus;
		static bool _can0;
		static bool _can1;
		
		CanControl(uint8_t);
	protected:
		CanControl(){};
};

#endif