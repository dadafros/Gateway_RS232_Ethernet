/************************ GATEWAY EPOS 70/10 MAXON MOTOR RS-232 -> ETHERNET ***********************
By Davi Frossard
mail to: davi_bf@outlook.com
November, 2018
Federal University of Minas Gerais
Belo Horizonte, Brazil

See Documentation for more details
**************************************************************************************************/

#include <OPC.h>
#include <Bridge.h>
#include <Ethernet.h>
#include <SPI.h>
#include <Arduino_FreeRTOS.h>
#include "FreeRTOSConfig.h"
#include "epos.h"

//Set network configuration
OPCEthernet myArduinoMEGA;
byte mac[] = { 0x90, 0xA2, 0xDA, 0x0E, 0xAD, 0x8D };
IPAddress ip(192, 168, 0, 104);
const int listen_port = 80;

//OPC Variables
float posicao, posicao2, sp_posicao, velocidade, velocidade2, sp_velocidade;
bool start = false;
bool stop = false;
bool reset = false;
bool home = false;
bool modo = false; 
bool sel_mesa = false;

//  --- Utility functions --- //

static uint16_t pack_le_uint16(uint8_t *buf) {
	uint16_t ret = buf[0];
	ret += (uint16_t)buf[1] << 8;
	return ret;
}

static uint32_t pack_le_uint32(uint8_t *buf) {
	uint32_t ret = buf[0];
	ret += (uint32_t)buf[1] << 8;
	ret += (uint32_t)buf[2] << 16;
	ret += (uint32_t)buf[3] << 24;
	return ret;
}

//  --- Protocol functions --- //

static uint16_t crc_byte(uint16_t crc, uint8_t data) {
	int i;
	crc ^= (uint16_t)data << 8;
	for (i = 0; i < 8; i++)
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	return crc;
}

static uint16_t crc_data(uint16_t crc, uint8_t* data, int len) {
	uint8_t low_byte, high_byte;
	len -= (len % 2); //For safety.
	while (len > 0) {
		low_byte = *data++;
		high_byte = *data++;
		crc = crc_byte(crc, high_byte);
		crc = crc_byte(crc, low_byte);
		len -= 2;
	}
	return crc;
}

int send_frame(uint8_t opcode, size_t len, uint8_t *data, int sel_mesa) {
	
	uint8_t ready_ack[1], end_ack[1];
	uint8_t len_minus_1 = len / 2 - 1;
	uint16_t crc = crc_byte(0, opcode); // CRC calculation and transmission
	crc = crc_byte(crc, len_minus_1);
	crc = crc_data(crc, data, len);
	uint8_t crc_bytes[2];
	crc_bytes[0] = crc;
	crc_bytes[1] = crc >> 8;

	if (sel_mesa == 0) {
		Serial1.write(opcode); // Opcode transmission

		// Ready to transmit data?
		if (Serial1.readBytes(ready_ack, 1) == 0) {
			Serial.println("[EPOS_Inf] Timeout receiving ready ack.");
			return FAIL;
		}
		if (ready_ack[0] != 'O') {
			Serial.print("[EPOS_Inf] Unrecognized ready ack received: ");
			Serial.println(ready_ack[0], HEX);
			return FAIL;
		}

		// Lenght transmission
		Serial1.write(len_minus_1);

		int written = Serial1.write(data, len); // Data transmission
		if (written != len) {
			Serial.println("[EPOS_Inf] Error writing data.");
			return FAIL;
		}

		written = Serial1.write(crc_bytes, 2);
		if (written != 2) {
			Serial.println("[EPOS_Inf] Error writing CRC.");
			return FAIL;
		}

		// Transmission OK?
		if (Serial1.readBytes(end_ack, 1) == 0) {
			Serial.println("[EPOS_Inf] Timeout receiving end ack.");
			return FAIL;
		}
		if (end_ack[0] != 'O') {
			Serial.print("[EPOS_Inf] acknowledged error in reception: ");
			Serial.println(end_ack[0], HEX);
			return FAIL;
		}
		return SUCCESS;
	}
	else if (sel_mesa == 1) {
		Serial2.write(opcode); // Opcode transmission

		// Ready to transmit data?
		if (Serial2.readBytes(ready_ack, 1) == 0) {
			Serial.println("[EPOS_Sup] Timeout receiving ready ack.");
			return FAIL;
		}
		if (ready_ack[0] != 'O') {
			Serial.print("[EPOS_Sup] Unrecognized ready ack received: ");
			Serial.println(ready_ack[0], HEX);
			return FAIL;
		}

		// Lenght transmission
		Serial2.write(len_minus_1);

		int written = Serial2.write(data, len); // Data transmission
		if (written != len) {
			Serial.println("[EPOS_Sup] Error writing data.");
			return FAIL;
		}

		written = Serial2.write(crc_bytes, 2);
		if (written != 2) {
			Serial.println("[EPOS_Sup] Error writing CRC.");
			return FAIL;
		}

		// Transmission OK?
		if (Serial2.readBytes(end_ack, 1) == 0) {
			Serial.println("[EPOS_Sup] Timeout receiving end ack.");
			return FAIL;
		}
		if (end_ack[0] != 'O') {
			Serial.print("[EPOS_Sup] acknowledged error in reception: ");
			Serial.println(end_ack[0], HEX);
			return FAIL;
		}
		return SUCCESS;
	}
}

int recv_frame(size_t len, uint8_t *data, int sel_mesa) {
	uint8_t opcode[1];
	uint8_t len_minus_1[1];
	uint8_t recv_crc[2];
	uint16_t crc;
	
	if (sel_mesa == 0) {
		// Opcode reception
		if (Serial1.readBytes(opcode, 1) == 0) {
			Serial.println("[EPOS_Inf] Timeout response opcode.");
			return FAIL;
		}
		if (opcode[0]) {
			Serial.print("[EPOS_Inf] Invalid (non-null) response opcode: ");
			Serial.println(opcode[0], HEX);
			return FAIL;
		}

		Serial1.write('O'); // Ready to receive data

		// Lenght reception
		if (Serial1.readBytes(len_minus_1, 1) == 0) {
			Serial.println("[EPOS_Inf] Timeout response message length.");
			return FAIL;
		}
		if (len_minus_1[0] != len / 2 - 1) {
			Serial.print("[EPOS_Inf] Invalid response message length: ");
			Serial.println(len_minus_1[0], HEX);
			return FAIL;
		}

		if (Serial1.readBytes(data, len) == 0) { // Data reception
			Serial.println("[EPOS_Inf] Timeout waiting for message data.");
			return FAIL;
		}

		// CRC reception and calculation
		if (Serial1.readBytes(recv_crc, 2) == 0) {
			Serial.println("[EPOS_Inf] Timeout waiting for CRC.");
			return FAIL;
		}
		crc = crc_byte(0, opcode[0]);
		crc = crc_byte(crc, len_minus_1[0]);
		crc = crc_data(crc, data, len);
		if (crc != pack_le_uint16(recv_crc)) {
			Serial1.write('F'); // Reception NOT OK
			Serial.println("[EPOS_Inf] Invalid message CRC received.");
			return FAIL;
		}

		Serial1.write('O'); // Reception OK
		return SUCCESS;
	}
	else if (sel_mesa == 1) {
		// Opcode reception
		if (Serial2.readBytes(opcode, 1) == 0) {
			Serial.println("[EPOS_Sup] Timeout response opcode.");
			return FAIL;
		}
		if (opcode[0]) {
			Serial.print("[EPOS_Sup] Invalid (non-null) response opcode: ");
			Serial.println(opcode[0], HEX);
			return FAIL;
		}

		Serial2.write('O'); // Ready to receive data

		// Lenght reception
		if (Serial2.readBytes(len_minus_1, 1) == 0) {
			Serial.println("[EPOS_Sup] Timeout response message length.");
			return FAIL;
		}
		if (len_minus_1[0] != len / 2 - 1) {
			Serial.print("[EPOS_Sup] Invalid response message length: ");
			Serial.println(len_minus_1[0], HEX);
			return FAIL;
		}

		if (Serial2.readBytes(data, len) == 0) { // Data reception
			Serial.println("[EPOS_Sup] Timeout waiting for message data.");
			return FAIL;
		}

		// CRC reception and calculation
		if (Serial2.readBytes(recv_crc, 2) == 0) {
			Serial.println("[EPOS_Sup] Timeout waiting for CRC.");
			return FAIL;
		}
		crc = crc_byte(0, opcode[0]);
		crc = crc_byte(crc, len_minus_1[0]);
		crc = crc_data(crc, data, len);
		if (crc != pack_le_uint16(recv_crc)) {
			Serial2.write('F'); // Reception NOT OK
			Serial.println("[EPOS_Sup] Invalid message CRC received.");
			return FAIL;
		}

		Serial2.write('O'); // Reception OK
		return SUCCESS;
	}
}

//  --- Exported module functions --- //

int epos_read_object(uint16_t index, uint8_t subindex, uint8_t nodeid, int sel_mesa, float *value_ptr) {
	// Clean Buffer
	if (sel_mesa==0)
		while (Serial1.available() > 0)
			Serial1.read();
	else if (sel_mesa == 1)
		while (Serial2.available() > 0)
			Serial2.read();

	uint8_t request[4];
	request[0] = index;
	request[1] = index >> 8;
	request[2] = subindex;
	request[3] = nodeid;

	send_frame(READ_OBJECT_OPCODE, sizeof request, request, sel_mesa);
	uint8_t response[8];
	recv_frame(sizeof response, response, sel_mesa);

	uint32_t error = pack_le_uint32(response);
	if (error) {
		Serial.print("Error in ReadObject ");
		Serial.println(error, HEX);
		return FAIL;
	}
	*value_ptr = (int32_t)pack_le_uint32(response + 4);

	return SUCCESS;
}

int epos_write_object(uint16_t index, uint8_t subindex, uint8_t nodeid, int sel_mesa, uint32_t value) {
	// Clean Buffer
	if (sel_mesa == 0)
		while (Serial1.available() > 0)
			Serial1.read();
	else if (sel_mesa == 1)
		while (Serial2.available() > 0)
			Serial2.read();

	uint8_t request[8];
	request[0] = index;
	request[1] = index >> 8;
	request[2] = subindex;
	request[3] = nodeid;
	request[4] = value;
	request[5] = value >> 8;
	request[6] = value >> 16;
	request[7] = value >> 24;

	send_frame(WRITE_OBJECT_OPCODE, sizeof request, request, sel_mesa);
	uint8_t response[4];
	recv_frame(sizeof response, response, sel_mesa);

	uint32_t error = pack_le_uint32(response);
	if (error) {
		Serial.print("Error in WriteObject ");
		Serial.println(error);
		return FAIL;
	}

	return SUCCESS;
}

void epos_halt(int sel_mesa) {
	epos_write_object(EPOS_CONTROL_WORD_INDEX, 0, 0, sel_mesa, EPOS_HALT_CMD);
}

void init_cmd(int sel_mesa) {
	epos_write_object(EPOS_CONTROL_WORD_INDEX, 0, 0, sel_mesa, EPOS_FAULT_RESET_CMD);
	vTaskDelay(pdMS_TO_TICKS(10));

	epos_write_object(EPOS_CONTROL_WORD_INDEX, 0, 0, sel_mesa, EPOS_SHUTDOWN_CMD);
	vTaskDelay(pdMS_TO_TICKS(10));

	epos_write_object(EPOS_CONTROL_WORD_INDEX, 0, 0, sel_mesa, EPOS_SWITCH_ON_CMD);
	vTaskDelay(pdMS_TO_TICKS(10));

	epos_write_object(EPOS_CONTROL_WORD_INDEX, 0, 0, sel_mesa, EPOS_ENABLE_OPERATION_CMD);
	vTaskDelay(pdMS_TO_TICKS(10));
}

void pos_cmd(int sel_mesa, uint32_t pos) {
	epos_write_object(EPOS_MODES_OPERATION_INDEX, 0, 0, sel_mesa, EPOS_PROFILE_POSITION_MODE);
	vTaskDelay(pdMS_TO_TICKS(10));

	epos_write_object(EPOS_TARGET_POSITION_INDEX, 0, 0, sel_mesa, pos);
	vTaskDelay(pdMS_TO_TICKS(10));

	epos_write_object(EPOS_CONTROL_WORD_INDEX, 0, 0, sel_mesa, EPOS_GOTO_POSITION_ABS_CMD);
	vTaskDelay(pdMS_TO_TICKS(10));
}

void vel_cmd(int sel_mesa, uint32_t vel) {
	epos_write_object(EPOS_MODES_OPERATION_INDEX, 0, 0, sel_mesa, EPOS_PROFILE_VELOCITY_MODE);
	vTaskDelay(pdMS_TO_TICKS(10));

	epos_write_object(EPOS_TARGET_VELOCITY_INDEX, 0, 0, sel_mesa, vel);
	vTaskDelay(pdMS_TO_TICKS(10));

	epos_write_object(EPOS_CONTROL_WORD_INDEX, 0, 0, sel_mesa, EPOS_GOTO_VELOCITY_CMD);
	vTaskDelay(pdMS_TO_TICKS(10));
}

bool callback_modo(const char *itemID, const opcOperation opcOP, bool value) {
	if (opcOP == opc_opread)
		return modo;
	else if (opcOP == opc_opwrite)
		modo = value;
}

bool callback_sel_mesa(const char *itemID, const opcOperation opcOP, bool value) {
	if (opcOP == opc_opread)
		return sel_mesa;
	else if (opcOP == opc_opwrite)
		sel_mesa = value;
}

bool callback_start(const char *itemID, const opcOperation opcOP, bool value) {
	if (opcOP == opc_opread)
		return start;
	else if (opcOP == opc_opwrite)
		start = value;
}

bool callback_stop(const char *itemID, const opcOperation opcOP, bool value) {
	if (opcOP == opc_opread)
		return stop;
	else if (opcOP == opc_opwrite)
		stop = value;
}

bool callback_reset(const char *itemID, const opcOperation opcOP, bool value) {
	if (opcOP == opc_opread)
		return reset;
	else if (opcOP == opc_opwrite)
		reset = value;
}

bool callback_home(const char *itemID, const opcOperation opcOP, bool value) {
	if (opcOP == opc_opread)
		return home;
	else if (opcOP == opc_opwrite)
		home = value;
}

float callback_sp_posicao(const char *itemID, const opcOperation opcOP, float value) {
	if (opcOP == opc_opread)
		return sp_posicao;
	else if (opcOP == opc_opwrite)
		sp_posicao = value;
}

float callback_sp_velocidade(const char *itemID, const opcOperation opcOP, float value) {
	if (opcOP == opc_opread)
		return sp_velocidade;
	else if (opcOP == opc_opwrite)
		sp_velocidade = value;
}

float callback_posicao(const char *itemID, const opcOperation opcOP, float value) {
	return posicao;
}

float callback_velocidade(const char *itemID, const opcOperation opcOP, float value) {
	return velocidade;
}

float callback_posicao2(const char *itemID, const opcOperation opcOP, float value) {
	return posicao2;
}

float callback_velocidade2(const char *itemID, const opcOperation opcOP, float value) {
	return velocidade2;
}

/****************************
Task to communicate with EPOS
Period: 1s
****************************/
void TaskEPOS(void *pvParameters __attribute__((unused)))
{
	init_cmd(0);
	init_cmd(1);
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		//Read Position and Velocity
		epos_read_object(EPOS_POSITION_ACTUAL_VALUE_INDEX, 0x00, 0, 0, &posicao);
		epos_read_object(EPOS_VELOCITY_ACTUAL_VALUE_INDEX, 0x00, 0, 0, &velocidade);

		epos_read_object(EPOS_POSITION_ACTUAL_VALUE_INDEX, 0x00, 0, 1, &posicao2);
		epos_read_object(EPOS_VELOCITY_ACTUAL_VALUE_INDEX, 0x00, 0, 1, &velocidade2);

		//Send to Serial Port 0 the status of connection
		Serial.println("EPOS connections OK");

		//Process Commands
		if (start == true) {
			if (modo == 0)
				pos_cmd(sel_mesa, sp_posicao);
			if (modo == 1)
				vel_cmd(sel_mesa, sp_velocidade);
			start = false;
		}
		if (stop == true) {
			epos_halt(0);
			epos_halt(1);
			stop = false;
		}
		if (reset == true) {
			init_cmd(0);
			init_cmd(1);
			reset = false;
		}
		if (home == true) {
			pos_cmd(0, 0);
			pos_cmd(1, 0);
			home = false;
		}
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
	}
}

/***************************
Task to communicate via OPC.
Period: 50ms
***************************/
void TaskOPC(void *pvParameters __attribute__((unused)))
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	for (;;)
	{
		myArduinoMEGA.processOPCCommands();
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(30));
	}
}

void setup() {
	Serial.begin(115200);
	Serial1.begin(115200);
	Serial2.begin(115200);
	Serial1.setTimeout(500);
	Serial2.setTimeout(500);

	/************************************** OPC *****************************************/
	myArduinoMEGA.setup(listen_port, mac, ip);
	//Select Mode of Operation
	myArduinoMEGA.addItem("modo", opc_readwrite, opc_bool, callback_modo);
	//Select EPOS 1 or 2
	myArduinoMEGA.addItem("sel_mesa", opc_readwrite, opc_bool, callback_sel_mesa);
	//Start Operation
	myArduinoMEGA.addItem("start", opc_readwrite, opc_bool, callback_start);
	//Stop Operation
	myArduinoMEGA.addItem("stop", opc_readwrite, opc_bool, callback_stop);
	//EPOS Reset 
	myArduinoMEGA.addItem("reset", opc_readwrite, opc_bool, callback_reset);
	//EPOS Home 
	myArduinoMEGA.addItem("home", opc_readwrite, opc_bool, callback_home);
	//Position
	myArduinoMEGA.addItem("posicao", opc_read, opc_float, callback_posicao);
	//Velocity
	myArduinoMEGA.addItem("velocidade", opc_read, opc_float, callback_velocidade);
	//Position2
	myArduinoMEGA.addItem("posicao2", opc_read, opc_float, callback_posicao2);
	//Velocity2
	myArduinoMEGA.addItem("velocidade2", opc_read, opc_float, callback_velocidade2);
	//Position Setpoint
	myArduinoMEGA.addItem("sp_posicao", opc_readwrite, opc_float, callback_sp_posicao);
	//Velocity Setpoint
	myArduinoMEGA.addItem("sp_velocidade", opc_readwrite, opc_float, callback_sp_velocidade);
	/************************************************************************************/

	xTaskCreate(TaskEPOS, (const portCHAR *)"EPOS", 1024, NULL, 2, NULL);
	xTaskCreate(TaskOPC, (const portCHAR *) "OPC", 1024, NULL, 3, NULL);
}

void loop()
{
	//Nothing to do here. Things are done in tasks
}