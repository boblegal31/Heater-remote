
extern void rflink_encode_oregon_thr (uint16_t id, int16_t temp, uint8_t hum, unsigned batstat);

enum ViessmannId {
	TempInterneChaudiere,
	TempEauChaudeSanitaire,
	TempConsigne,
	TempExterieure,
	TempDepartCircuit,
	StatusCommandMode,
	StatusOrganes,
	StatusPompes,
	ConfigureInterface
};

enum ViessmannModes {
	ModeOff,
	ModeEco,
	ModeParty,
	ModeJour,
	ModeNuit,
	ConsigneNormal,
	ConsigneReduit,
	AjusterHeure,
};

enum InterfaceConfig {
	SetTempSensorId,
	SetCircuitNumber
};

enum ViessmannOrganes {
	Bruleur,
	Pompe_chaudiere,
	Pompe_bouclage,
	bit0,
	bit1,
	bit3,
	bit4,
	bit5,
};

extern void rflink_encode_viessmann_temp (uint8_t circuit, uint8_t temp_id, uint16_t temp);
extern void rflink_encode_viessmann_status ( uint8_t circuit, uint8_t status_type, uint8_t status_id, uint8_t value);
