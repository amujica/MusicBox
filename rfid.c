#include "rfid.h"

void initialize_rfid(){
	MFRC522_Status_t ret;

	ret = MFRC522_Init('A');
		if (ret < 0) {
			perror("Error al inicializar RFID");
		}
}

char* read_id(){
	MFRC522_Status_t ret;
	uint8_t CardID[5] = { 0x00, };
	char* id_serial = (char*)malloc(9*sizeof(char));

	//puts("Scanning\n");
		ret = MFRC522_Check(CardID);
		if (ret != MI_OK) {
			return "00000000";
		}
		snprintf(id_serial, 9, "%02X%02X%02X%02X\n",CardID[0], CardID[1], CardID[2], CardID[3]);
		printf("ID leido: %s\n", id_serial);
	return id_serial;
}
