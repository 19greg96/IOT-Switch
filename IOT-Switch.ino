
extern "C" {
	#include "c_types.h"
	#include "ets_sys.h"
	#include "os_type.h"
	#include "osapi.h"
	#include "spi_flash.h"
	#include <user_interface.h>
	
	#include <stdio.h>
}

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

#include <ESPAsyncTCP.h>
#include <SyncClient.h>

#include <EEPROM.h>

const char* ssid = "***"; // router ssid
const char* password = "***"; // router wifi password

#define LED_PIN 2

#define EEPROM_SIZE								128
#define	EEPROM_VAL_MEM_COHERENCY_FLAG			85			// TODO: this should be calculated from CRC
#define	EEPROM_ADDR_1_MEM_COHERENCY_FLAG		0
#define	EEPROM_ADDR_4_LED_INTENSITY				EEPROM_ADDR_1_MEM_COHERENCY_FLAG + 1
#define	EEPROM_ADDR_1_SET_INTENSITY_ON_BOOT		EEPROM_ADDR_4_LED_INTENSITY + 4
#define	EEPROM_ADDR_33_HOSTNAME					EEPROM_ADDR_1_SET_INTENSITY_ON_BOOT + 1		// +1 byte for zero ending
#define	EEPROM_ADDR_6_MAC_ADDR					EEPROM_ADDR_33_HOSTNAME + 33

void ICACHE_RAM_ATTR ramMeasureTimerISR();

int ledIntensity = 0;
char setIntensityOnBoot = 0;

unsigned long OStime = 0;		// value of accumulated OS time over 1s
unsigned long OStimeAcc = 0;	// OS time accumulator, reset every 1s
unsigned long OSStartTime;

unsigned long userTime = 0;		// aggregateInterval - OStime

unsigned long lastAggregateTime; // aggregate statistics every 1s
unsigned long aggregateInterval = 1000000; // aggregate statistics every 1s
unsigned long lastPingTime;

#define NUM_SAMPLES		128
#define MAX_SAMPLES		(NUM_SAMPLES - 1)
volatile uint32_t freeRamCircBuff[128] = {0};
volatile uint32_t freeRamCircBuffPtr = 0;
volatile uint32_t freeRamRollingAvarageSum = 0;

extern "C" void __yield();
void yield() { // overrite default yield function, to measure system time
	OSStartTime = micros();
	__yield();
	OStimeAcc += micros() - OSStartTime;
}

void wifiConnect();

ADC_MODE(ADC_VCC);

ESP8266WebServer server(80);
SyncClient ping_client;

void EEPROM_write_uint32(int addr, int x) {
	EEPROM.write(addr + 0, x & 0xFF);
	EEPROM.write(addr + 1, (x >> 8) & 0xFF);
	EEPROM.write(addr + 2, (x >> 16) & 0xFF);
	EEPROM.write(addr + 3, (x >> 24) & 0xFF);
}
int EEPROM_read_uint32(int addr) {
	return (EEPROM.read(addr + 1) |
			(EEPROM.read(addr + 1) << 8) |
			(EEPROM.read(addr + 2) << 16) |
			(EEPROM.read(addr + 3) << 24));
}

void EEPROM_write_uint8(int addr, char x) {
	EEPROM.write(addr + 0, x & 0xFF);
}
char EEPROM_read_uint8(int addr) {
	return EEPROM.read(addr + 0);
}

void EEPROM_write_uint8_arr(int addr, const char* x, int len) {
	for (int i = 0; i < len; i ++) {
		EEPROM.write(addr + i, x[i] & 0xFF);
	}
}
void EEPROM_read_uint8_arr(int addr, char* out, int len) {
	for (int i = 0; i < len; i ++) {
		*(out + i) = EEPROM.read(addr + i);
	}
}
void EEPROM_lock() {
	EEPROM_write_uint8(EEPROM_ADDR_1_MEM_COHERENCY_FLAG, 0);
	EEPROM.commit();
}
void EEPROM_unlock() {
	EEPROM_write_uint8(EEPROM_ADDR_1_MEM_COHERENCY_FLAG, EEPROM_VAL_MEM_COHERENCY_FLAG);
	EEPROM.commit();
}

uint64_t millis64() {
	static uint32_t low32, high32;
	uint32_t new_low32 = millis();
	if (new_low32 < low32) high32++;
	low32 = new_low32;
	return (uint64_t) high32 << 32 | low32;
}

void setLedIntensity(int x) {
	ledIntensity = constrain(x, 0, 1024);
	
	EEPROM_lock();
	EEPROM_write_uint32(EEPROM_ADDR_4_LED_INTENSITY, ledIntensity);
	EEPROM.commit();
	EEPROM_unlock();
	
	//analogWrite(LED_PIN, ledIntensity);
	if (ledIntensity == 0) { // PWM temporarily off, untill we get zero-cross detector
		digitalWrite(LED_PIN, 0);
		// analogWrite(LED_PIN, 0);
	} else {
		digitalWrite(LED_PIN, 1);
		// analogWrite(LED_PIN, 1024);
	}
}



int8_t charToNum(char chr) {
	// 48 - '0'
	// 65 - 'A'
	// 97 - 'a'
	if (chr >= '0' && chr <= '9') {
		return (chr - '0');
	}
	if (chr >= 'A' && chr <= 'F') {
		return (chr - 'A' + 10);
	}
	if (chr >= 'a' && chr <= 'f') {
		return (chr - 'a' + 10);
	}
	return -1;
}
bool readHex(int8_t chrMSB, int8_t chrLSB, unsigned char* out) {
	if ((chrMSB = charToNum(chrMSB)) == -1) {
		return false;
	}
	if ((chrLSB = charToNum(chrLSB)) == -1) {
		return false;
	}
	*out = ((chrMSB << 4) | chrLSB);
	return true;
}

void passiveDelay(unsigned long ms) {
	uint32_t start = micros();

	while (ms > 0) {
		yield();
		while ( ms > 0 && (micros() - start) >= 1000) {
			ms--;
			start += 1000;
		}
	}
}

void prettyBytes(char* buf, uint32_t bytes) {
	const char* suffixes[7];
	suffixes[0] = "B";
	suffixes[1] = "KB";
	suffixes[2] = "MB";
	suffixes[3] = "GB";
	suffixes[4] = "TB";
	suffixes[5] = "PB";
	suffixes[6] = "EB";
	uint32_t s = 0; // which suffix to use
	double count = bytes;
	while (count >= 1024 && s < 7) {
		s++;
		count /= 1024;
	}
	if (count - floor(count) == 0.0) {
		sprintf(buf, "%d %s", (int)count, suffixes[s]);
	} else {
		sprintf(buf, "%.1f %s", count, suffixes[s]);
	}
}
void prettyTime(char* buf, uint64_t timeMillis) {
	uint64_t seconds = timeMillis / 1000;
	uint32_t d = seconds / 86400;
	uint32_t h = (seconds % 86400) / 3600;
	uint32_t m = (seconds % 3600) / 60;
	uint32_t s = (seconds % 60);
	
	if (d > 0) {
		sprintf(buf, "%dd %dh %dm %ds", d, h, m, s);
	} else if (h > 0) {
		sprintf(buf, "%dh %dm %ds", h, m, s);
	} else if (m > 0) {
		sprintf(buf, "%dm %ds", m, s);
	} else {
		sprintf(buf, "%ds", s);
	}
}
const char* getBatteryIcon(float voltage) {
	// 1.7-3.6V
	// 0.2375
	float minVoltage = 1.7;
	float stepSize = 0.2375;
	if (voltage > minVoltage + stepSize && voltage <= minVoltage + stepSize * 2) {
		return "battery_20";
	} else if (voltage > minVoltage + stepSize * 2 && voltage <= minVoltage + stepSize * 3) {
		return "battery_30";
	} else if (voltage > minVoltage + stepSize * 3 && voltage <= minVoltage + stepSize * 4) {
		return "battery_50";
	} else if (voltage > minVoltage + stepSize * 4 && voltage <= minVoltage + stepSize * 5) {
		return "battery_60";
	} else if (voltage > minVoltage + stepSize * 5 && voltage <= minVoltage + stepSize * 6) {
		return "battery_80";
	} else if (voltage > minVoltage + stepSize * 6 && voltage <= minVoltage + stepSize * 7) {
		return "battery_90";
	} else if (voltage > minVoltage + stepSize * 7 && voltage <= minVoltage + stepSize * 8) {
		return "battery_full";
	}
	return "battery_alert";
}
void getRamInfo(uint32_t* dataSize, uint32_t* ramSize, uint32_t* freeRam, uint32_t* fullRam) {
	// See: eboot.ld, eagle.app.v6.common.ld
	extern char _data_start;	// 0x3ffe8000
	//extern char _data_end;		// 0x3ffe8518	data size = 1304
	//extern char _rodata_start;	// 0x3ffe8520	offset from data start = 1312
	//extern char _rodata_end;	// 0x3ffebeb8	rodata size = 14744; offset from data start = 16056
	//extern char _bss_start;		// 0x3ffebeb8	offset from data start = 16056
	//extern char _bss_end;		// 0x3fff3320	bss size = 29800;  offset from data start = 45856
	extern char _heap_start;	// 0x3fff3320
	// dram0_0_seg :                       	org = 0x3FFE8000, len = 0x14000
	// (_heap_start - _data_start) = Global variables
	// Global variables use 45848 bytes (55%) of dynamic memory, leaving 36072 bytes for local variables. Maximum is 81920 bytes.
	
	uint32_t dramSize = 81920;
	*dataSize = &_heap_start - &_data_start; // size of Global variables
	*ramSize = dramSize - *dataSize; // local variables, "Heap size"
	*freeRam = ESP.getFreeHeap(); // >> 7; // division by 128, which is number of samples. //ESP.getFreeHeap();
	*fullRam = *ramSize - *freeRam;
}
const char* getChipName() {
	uint32_t chipId = ESP.getFlashChipId();
	 // * Chip ID
	 // * 00 - always 00 (Chip ID use only 3 byte)
	 // * 17 - ? looks like 2^xx is size in Byte ?     //todo: find docu to this
	 // * 40 - ? may be Speed ?                        //todo: find docu to this
	 // * C8 - manufacturer ID
	switch(chipId) {

		// GigaDevice
		case 0x1740C8:
			return "GD25Q64B";
		case 0x1640C8:
			return "GD25Q32B";
		case 0x1540C8:
			return "GD25Q16B";
		case 0x1440C8:
			return "GD25Q80";
		case 0x1340C8:
			return "GD25Q40";
		case 0x1240C8:
			return "GD25Q20";
		case 0x1140C8:
			return "GD25Q10";
		case 0x1040C8:
			return "GD25Q12";

		// Winbond
		case 0x1640EF:
			return "W25Q32";
		case 0x1540EF:
			return "W25Q16";
		case 0x1440EF:
			return "W25Q80";
		case 0x1340EF:
			return "W25Q40";

		// BergMicro
		case 0x1640E0:
			return "BG25Q32";
		case 0x1540E0:
			return "BG25Q16";
		case 0x1440E0:
			return "BG25Q80";
		case 0x1340E0:
			return "BG25Q40";

		default:
			return "Unknown";
	}
}



void sendDeviceRestartPage(String message) {
	server.send(200, "text/html", "<meta http-equiv='refresh' content='7; url=/' />" + message + "<br/>Restarting...");
	passiveDelay(1000);
	server.stop();
	passiveDelay(1000);
	
	ESP.restart();
}
void handleNotFound() {
	String message = "File Not Found\n\n";
	message += "URI: ";
	message += server.uri();
	message += "\nMethod: ";
	message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
	message += "\nArguments: ";
	message += server.args();
	message += "\n";

	for ( uint8_t i = 0; i < server.args(); i++ ) {
		message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
	}

	server.send ( 404, "text/plain", message );
}
void handleIntensity() {
	if (server.hasArg("led_intensity")) {
		setLedIntensity(server.arg("led_intensity").toInt());
	}
	
	server.send(200, "text/html", String(ledIntensity));
}
void handleToggle() {
	setLedIntensity(!ledIntensity);
	
	server.send(200, "text/html", String(ledIntensity));
}
void handleConfig() {
	if (server.hasArg ("set_intensity_on_boot")) {
		setIntensityOnBoot = 1;
	} else {
		setIntensityOnBoot = 0;
	}
	EEPROM_lock();
	EEPROM_write_uint8(EEPROM_ADDR_1_SET_INTENSITY_ON_BOOT, setIntensityOnBoot);
	EEPROM.commit();
	EEPROM_unlock();
	
	server.sendHeader("Location", "/");
	server.send(303, "text/html", ""); // 303 See Other (to force GET when the browser requests /)
}
void handleInfo() {
	String reply = "{";
	
	char strHolder[32];
	prettyTime(strHolder, millis64());
	reply += "\"uptime\": \"" + String(strHolder) + "\", ";
	
	uint32_t dataSize;
	uint32_t ramSize;
	uint32_t freeRam;
	uint32_t fullRam;
	getRamInfo(&dataSize, &ramSize, &freeRam, &fullRam);
	String strRamPercent = String(((float)fullRam / ramSize) * 100.0);
	prettyBytes(strHolder, freeRam);
	reply += "\"free_heap\": \"" + String(strHolder) + "\", ";
	reply += "\"heap_progress\": \"&nbsp;&nbsp;";
	prettyBytes(strHolder, fullRam);
	reply += String(strHolder) + " / ";
	prettyBytes(strHolder, ramSize);
	reply += String(strHolder) + "\", ";
	reply += "\"heap_percent\": \"" + strRamPercent + "\", ";
	
	uint32_t Vcc = ESP.getVcc();
	reply += "\"vcc\": \"" + String(Vcc / 1024.0f, 3) + "V\", ";
	reply += "\"vcc_icon\": \"" + String(getBatteryIcon(Vcc / 1024.0f)) + "\", ";
	
	reply += "\"rssi\": \"" + String(WiFi.RSSI()) + "\", ";
	
	reply += "\"os_time\": \"" + String(OStime) + "uS\", ";
	
	reply += "\"loop_time\": \"" + String(userTime) + "uS\", ";
	
	reply += "\"intensity\": \"" + String(ledIntensity) + "\"";
	
	reply += "}";
	
	server.send(200, "application/json", String(reply));
}

void handleNetConfig() {
	char hostname[33];
	char strMacAddr[18];
	
	strcpy(hostname, server.arg("hostname").c_str());
	strcpy(strMacAddr, server.arg("macaddr").c_str());
	
	if (strlen(hostname) > 32) {
		server.send(200, "text/html", "<meta http-equiv='refresh' content='5; url=/' />Hostname too long (max is 32)");
		return;
	}
	
	unsigned char mac[6];
	if (strlen(strMacAddr) != 17) {
		server.send(200, "text/html", "<meta http-equiv='refresh' content='5; url=/' />Invalid mac address format (length not 17)");
		return;
	}
	for (int i = 0, pos = 0; i < 6; i ++) {
		if (!readHex(strMacAddr[pos], strMacAddr[pos + 1], &(mac[i])) || (strMacAddr[pos + 2] != ':' && i != 5)) {
			server.send(200, "text/html", "<meta http-equiv='refresh' content='5; url=/' />Invalid mac address format i=" + String(i) + " pos=" + String(pos) + " chr0=" + String(strMacAddr[pos]) + " chr1=" + String(strMacAddr[pos + 1]) + " chr2=" + String(strMacAddr[pos + 2]));
			return;
		}
		pos += 3;
	}
	if (!wifi_set_macaddr(STATION_IF, mac)) {
		server.send(200, "text/html", "<meta http-equiv='refresh' content='5; url=/' />Failed to set mac address. (Probably Unicast vs. multicast)");
		return;
	}
	// Set this after everything else succeedes
	WiFi.hostname(hostname);
	
	EEPROM_lock();
	EEPROM_write_uint8_arr(EEPROM_ADDR_33_HOSTNAME, hostname, 33);
	EEPROM_write_uint8_arr(EEPROM_ADDR_6_MAC_ADDR, (const char*)mac, 6);
	EEPROM.commit();
	EEPROM_unlock();
	
	sendDeviceRestartPage("");
}



void handleMain() {
	String serverIndex = "<!doctype html><html>";
	serverIndex += "<head>";
	serverIndex += "<meta name='viewport' content='width=device-width, initial-scale=1, shrink-to-fit=no'>";
	serverIndex += "<title>" + WiFi.hostname() + "</title>";
	serverIndex += "<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.1.0/jquery.min.js'></script>";
	serverIndex += "<link rel='stylesheet' href='https://fonts.googleapis.com/css?family=Roboto:300,400,500,700|Material+Icons'>";
	serverIndex += "<link rel='stylesheet' href='https://unpkg.com/bootstrap-material-design@4.1.1/dist/css/bootstrap-material-design.min.css' integrity='sha384-wXznGJNEXNG1NFsbm0ugrLFMQPWswR3lds2VeinahP8N0zJw9VWSopbjv2x7WCvX' crossorigin='anonymous'>";
	serverIndex += "<style>								"
".material-icons { vertical-align: text-bottom; }		"
"body { -webkit-touch-callout: none; -webkit-user-select: none; -khtml-user-select: none; -moz-user-select: none; -ms-user-select: none; user-select: none; }	"
".selectable,code { -webkit-touch-callout: text; -webkit-user-select: text; -khtml-user-select: text; -moz-user-select: text; -ms-user-select: text; user-select: text; }	"
".card { border-radius: 20px; }							"
".progress { height: 22px; white-space: nowrap; }							"
".card .card-header {padding: .75rem 1.25rem; }			"
".card-columns .card { display: block; white-space: nowrap; overflow: hidden; -webkit-column-break-inside: avoid; }	"
"@media (min-width:576px) { .card-columns { column-count:2; } } "
"@media (min-width:768px) { .card-columns { column-count:3; } }	"
"@media (min-width:1200px) { .card-columns { column-count:4; } }	"
".btn,.progress,.alert { border-radius: 4px; }	"
".lampSwitch{ font:15px/1 arial; text-align:center; border:2px solid #444444; border-radius:7px; display: block;		"
"			  vertical-align:middle; font-weight:bold; box-shadow: 1px 1px 1px 1px #000; width:100px; height:100px;		"
"			  position:relative; margin:0 5px; color:#4c4c4c; box-shadow:0 0 5px rgba(255,255,255,.4); } 				"
".lampSwitch input{ position:absolute; visibility:hidden; }																"
".lampSwitch > div{ background:linear-gradient(to bottom, #424242 0%, #777777 11%, #757575 14%, #f1f1f1 58%, #dbdbdb 96%, #f1f1f1 100%); "
"			 		height:100%; border-radius:5px; line-height:50px; position:relative; z-index:1; cursor:pointer; text-shadow:0 1px rgba(255,255,255,0.5); }	"
".lampSwitch > div:after { content:'O'; display:block; height:50%; line-height:4; transform-origin:0 0; }				"
".lampSwitch > div:before { content:'I'; display:block; height:50%; line-height:2.5; border-radius:80%/5px;				"
"							box-shadow:0 8px 12px -13px #bababa inset, 0 -2px 2px -1px rgba(255,255,255,0.8); transform-origin:0 100%; transform:scaleY(0.7); }	"
".lampSwitch :checked ~ div{ background:linear-gradient(to top, #424242 0%, #777777 11%, #757575 14%, #f1f1f1 58%, #dbdbdb 96%, #f1f1f1 100%); }	"
".lampSwitch :checked ~ div:before{ border-radius:0; box-shadow:none; transform:none; }									"
".lampSwitch :checked ~ div:after{ border-radius:80%/5px; box-shadow:0 -8px 12px -5px #bababa inset, 0 2px 2px 0 #565656; transform:scaleY(0.7); }	"
".lampSwitch .indicator{ position:absolute; top:-20px; left:36px; width:25px; height:8px; box-shadow:0 0 8px #000 inset;		"
"						 box-sizing: initial; border:1px solid rgba(255,255,255,0.1); border-radius:15px; transition:0.2s; }	"
".lampSwitch .indicator:before{ content:''; display:block; margin:2px auto; width:8px; height:5px; border-radius:10px; transition:0.5s; }	"
".lampSwitch :checked ~ .indicator:before{ box-shadow:0 0 2px 0px #F95757 inset, 0 0 12px 6px #F95757; background:#FFF; transition:0.1s; }	"
"</style>";
	serverIndex += "<script>";
	serverIndex += "																				\n"
	"var canRequest = true;																			\n"
	"var lastVal = " + String(ledIntensity) + ";													\n"
	"var lampState = " + String(ledIntensity) + ";													\n"
	"$(document).ready(function() {																	\n"
	"	function setIntensity(val) {																\n"
	"		if (!canRequest) {																		\n"
	"			return;																				\n"
	"		}																						\n"
	"		canRequest = false;																		\n"
	"		$.ajax({																				\n"
	"			type: 'POST',																		\n"
	"			dataType: 'text',																	\n"
	"			url: '/intensity',																	\n"
	"			data: {'led_intensity': val},														\n"
	"			success: function (e) {																\n"
	"				lampState = e;																	\n"
	"				$('#btn_lamp_toggle').prop('checked', (lampState == 0));						\n"
	// "				$('#intensity_value').val(e);													\n"
	// "				$('#intensity_slider').val(e);													\n"
	"				$('#btn_lamp_toggle').prop('disabled', false);									\n"
	"				canRequest = true;																\n"
	"			}																					\n"
	"		});																						\n"
	"	}																							\n"
	"	(function updateInfo() {																	\n"
	"		$.ajax({																				\n"
	"			type: 'POST',																		\n"
	"			dataType: 'json',																	\n"
	"			url: '/info',																		\n"
	"			data: {},																			\n"
	"			success: function (e) {																\n"
	"				$('#uptimeSpan').html(e.uptime);												\n"
	"				$('#freeHeapSpan').html(e.free_heap);											\n"
	"				$('#heapProgressBar').html(e.heap_progress);									\n"
	"				$('#heapProgressBar').attr('aria-valuenow', e.heap_percent + '%')				\n"
	"				$('#heapProgressBar').css('width', e.heap_percent + '%');						\n"
	"				$('#vccSpan').html(e.vcc);														\n"
	"				$('#vccIcon').html(e.vcc_icon);													\n"
	"				$('#rssiSpan').html(e.rssi);													\n"
	"				$('#osTimeSpan').html(e.os_time);												\n"
	"				$('#userTimeSpan').html(e.loop_time);											\n"
	"				if (canRequest) {																\n" // only update value, if there is no other request going on. (This should avoid a hazard)
	"					lampState = e.intensity;													\n"
	"					$('#btn_lamp_toggle').prop('checked', (lampState == 0));					\n"
	"				}																				\n"
	//"				$('#intensity_value').val(e.intensity);											\n"
	//"				$('#intensity_slider').val(e.intensity);										\n"
	"				setTimeout(updateInfo, 1000);													\n"
	"			}																					\n"
	"		});																						\n"
	"	})();																						\n"
	"	$('#btn_lamp_toggle').click(function() {													\n"
	"		$('#btn_lamp_toggle').prop('disabled', true);											\n"
	"		setIntensity(lampState == 0 ? 1024 : 0);												\n"
	"	});																							\n"
	// "	$('#intensity_slider').on('change mousemove', function () {									\n"
	// "		if (canRequest && lastVal != $('#intensity_slider').val()) {							\n"
	// "			lastVal = $('#intensity_slider').val();												\n"
	// "			setIntensity($('#intensity_slider').val());											\n"
	// "		}																						\n"
	// "	});																							\n"
	"});																							\n";
	serverIndex += "</script>";
	serverIndex += "</head>";
	serverIndex += "<body><div class='container'>";
	serverIndex += "<h1>" + WiFi.hostname() + "</h1>";
	
	// if (server.hasArg ("led_state")) {
		// if (server.arg("led_state") == "on") {
			// analogWrite(LED_PIN, ledIntensity);
			// serverIndex += "turning led on<br/>";
		// } else if (server.arg("led_state") == "off") {
			// analogWrite(LED_PIN, 0);
			// serverIndex += "turning led off<br/>";
		// } else {
			// serverIndex += "invalid param: " + server.arg("led_state") + "<br/>";
		// }
		// serverIndex += "<hr/>";
	// }
	
	char strHolder[32];
	uint32_t realSize = ESP.getFlashChipRealSize();
	uint32_t ideSize = ESP.getFlashChipSize();
	uint32_t Vcc = ESP.getVcc();
	FlashMode_t ideMode = ESP.getFlashChipMode();
	
	serverIndex += "<h5><i class='material-icons'>flash_on</i>&nbsp;Lamp switch</h5>";
	serverIndex += "<div class='card mb-3'><div class='card-body'>";
		serverIndex += "<label class='lampSwitch mt-3 mx-auto'><input id='btn_lamp_toggle' type='checkbox' " + String(ledIntensity == 0 ? "checked" : "") + "/><div></div><span class='indicator'></span></label>";
		//serverIndex += "<div class='switch'><label><input id='btn_lamp_toggle' type='checkbox' " + String(ledIntensity == 0 ? "checked" : "") + ">Lamp</label></div>";
		// serverIndex += "<input id='intensity_slider' type='range' min='0' max='1024' value='" + String(ledIntensity) + "'/><br/>";
		// serverIndex += "<input disabled id='intensity_value' type='text' value='" + String(ledIntensity) + "'/>";
	serverIndex += "</div></div>";
	
	serverIndex += "<h5><i class='material-icons'>info</i>&nbsp;ESP config</h5>";
	serverIndex += "<div class='card-columns'>";
		serverIndex += "<div class='card mb-3'>";
			serverIndex += "<div class='card-header'><h5 class='card-title mb-0'><small class='text-muted'><i class='material-icons'>sd_storage</i>&nbsp;Flash</small></h5></div>";
			serverIndex += "<div class='card-body'>";
			String strFullPercent = String(((float)ESP.getSketchSize() / (ESP.getFreeSketchSpace() + ESP.getSketchSize())) * 100);
			
			serverIndex += "<div class='progress mb-2'><div class='progress-bar bg-info' role='progressbar' style='width: " + strFullPercent + "%' aria-valuenow='" + strFullPercent + "' aria-valuemin='0' aria-valuemax='100'>";
				prettyBytes(strHolder, ESP.getSketchSize());
				serverIndex += "&nbsp;&nbsp;" + String(strHolder) + " / ";
				prettyBytes(strHolder, (ESP.getFreeSketchSpace() + ESP.getSketchSize()));
			serverIndex += String(strHolder) + "</div></div>";
			prettyBytes(strHolder, realSize);
			serverIndex += "Chip:<span class='float-right selectable'>" + String(getChipName()) + " - " + String(strHolder) + "</span><br/>";
			prettyBytes(strHolder, ideSize);
			serverIndex += "ide:<span class='float-right selectable'>" + String(strHolder) + " - " + String(ideMode == FM_QIO ? "QIO" : ideMode == FM_QOUT ? "QOUT" : ideMode == FM_DIO ? "DIO" : ideMode == FM_DOUT ? "DOUT" : "UNKNOWN") + " - " + String(ESP.getFlashChipSpeed() / 1000000) + "MHz</span>";
			if(ideSize != realSize) {
				serverIndex += "<div class='alert alert-warning mb-0 mt-2' role='alert'>Flash Chip configuration wrong!</div>";
			} else {
				serverIndex += "<div class='alert alert-success mb-0 mt-2' role='alert'>Flash Chip configuration ok.</div>";
			}
		serverIndex += "</div></div>";
		serverIndex += "<div class='card mb-3'>";
			serverIndex += "<div class='card-header'><h5 class='card-title mb-0'><small class='text-muted'><i class='material-icons'>access_time</i>&nbsp;Uptime</small></h5></div>";
			serverIndex += "<div class='card-body'>";
			prettyTime(strHolder, millis64());
			serverIndex += "Uptime:<span id='uptimeSpan' class='float-right'>" + String(strHolder) + "</span><br/>";
			/*
			Fatal exception:4 flag:3 (SOFT_WDT) epc1:0x4020130c epc2:0x00000000 epc3:0x00000000 excvaddr:0x00000000 depc:0x00000000
			Fatal exception:4 flag:3 (SOFT_WDT) epc1:0x4020130c epc2:0x00000000 epc3:0x00000000 excvaddr:0x00000000 depc:0x00000000
			*/
			serverIndex += "<div class='alert alert-info mb-0 mt-2' role='alert' data-toggle='tooltip' data-placement='top' data-html='true' title='<code style=\"color:#fff;\">" + ESP.getResetInfo() + "</code>'>";
				serverIndex += String(ESP.getResetReason());
			serverIndex += "</div>";
		serverIndex += "</div></div>";
		serverIndex += "<div class='card mb-3'>";
			serverIndex += "<div class='card-header'><h5 class='card-title mb-0'><small class='text-muted'><i class='material-icons'>memory</i>&nbsp;RAM</small></h5></div>";
			serverIndex += "<div class='card-body'>";
			
			uint32_t dataSize;
			uint32_t ramSize;
			uint32_t freeRam;
			uint32_t fullRam;
			getRamInfo(&dataSize, &ramSize, &freeRam, &fullRam);
			String strRamPercent = String(((float)fullRam / ramSize) * 100.0);
			
			serverIndex += "<div class='progress mb-2'><div id='heapProgressBar' class='progress-bar bg-info' role='progressbar' style='width: " + String(strRamPercent) + "%' aria-valuenow='" + String(strRamPercent) + "' aria-valuemin='0' aria-valuemax='100'>&nbsp;&nbsp;";
				prettyBytes(strHolder, fullRam);
				serverIndex += String(strHolder) + " / ";
				prettyBytes(strHolder, ramSize);
			serverIndex += String(strHolder) + "</div></div>";
			
			prettyBytes(strHolder, ramSize);
			serverIndex += "RAM size:<span class='float-right'>" + String(strHolder) + "</span><br/>";
			
			prettyBytes(strHolder, dataSize);
			serverIndex += "Global variables:<span class='float-right'>" + String(strHolder) + "</span><br/>";
			
			prettyBytes(strHolder, freeRam);
			serverIndex += "Free heap size:<span id='freeHeapSpan' class='float-right'>" + String(strHolder) + "</span><br/>";
			// serverIndex += "ISR call count:<span class='float-right'>" + String(freeRamCircBuffPtr) + "</span><br/>";
			// serverIndex += "Timer1 read:<span class='float-right'>" + String(timer1_read()) + "</span><br/>";
			// serverIndex += "Timer1 enabled:<span class='float-right'>" + String(timer1_enabled()) + "</span><br/>";
			// serverIndex += "Timer1 interrupted:<span class='float-right'>" + String(timer1_interrupted()) + "</span><br/>";
			// serverIndex += "Timer1 ISR address:<span class='float-right'>" + String((uint32_t)ramMeasureTimerISR) + "</span><br/>";
			
		serverIndex += "</div></div>";
		serverIndex += "<div class='card mb-3'>";
			serverIndex += "<div class='card-header'><h5 class='card-title mb-0'><small class='text-muted'><i id='vccIcon' class='material-icons'>" + String(getBatteryIcon(Vcc / 1024.0f)) + "</i>&nbsp;Voltage</small></h5></div>";
			serverIndex += "<div class='card-body'>";
			serverIndex += "Vcc:<span id='vccSpan' class='float-right'>" + String(Vcc / 1024.0f, 3) + "V</span>";
		serverIndex += "</div></div>";
		serverIndex += "<div class='card mb-3'>";
			serverIndex += "<div class='card-header'><h5 class='card-title mb-0'><small class='text-muted'><i class='material-icons'>wifi</i>&nbsp;Signal Strength</small></h5></div>";
			serverIndex += "<div class='card-body'>";
			serverIndex += "RSSI:<span id='rssiSpan' class='float-right'>" + String(WiFi.RSSI()) + "</span>";
		serverIndex += "</div></div>";
		serverIndex += "<div class='card mb-3'>";
			serverIndex += "<div class='card-header'><h5 class='card-title mb-0'><small class='text-muted'><i class='material-icons'>bar_chart</i>&nbsp;Load</small></h5></div>";
			serverIndex += "<div class='card-body'>";
			serverIndex += "CPU clock:<span class='float-right'>" + String(ESP.getCpuFreqMHz()) + "MHz</span><br/>";
			serverIndex += "OS time:<span id='osTimeSpan' class='float-right'>" + String(OStime) + "uS</span><br/>";
			serverIndex += "Loop time:<span id='userTimeSpan' class='float-right'>" + String(userTime) + "uS</span>";
		serverIndex += "</div></div>";
	serverIndex += "</div>";
	
	serverIndex += "<hr class='my-4'/>";
	
	serverIndex += "<div class='container'><div class='row'>";
		serverIndex += "<div class='col-md'><h5><i class='material-icons'>settings</i>&nbsp;Config</h5>";
		serverIndex += "<div class='card mb-3'><div class='card-body'>";
			serverIndex += "<form class='mb-0' method='POST' action='/config'>";
				serverIndex += "<div class='checkbox'><label><input type='checkbox' name='set_intensity_on_boot'" + String(setIntensityOnBoot ? " checked" : "") + "> Set last lamp state on boot</label></div>";
				serverIndex += "<button type='submit' class='btn btn-primary btn-raised'>Save</button>";
			serverIndex += "</form>";
		serverIndex += "</div></div></div>";
		
		serverIndex += "<div class='col-md'><h5><i class='material-icons'>build</i>&nbsp;Network config</h5>";
		serverIndex += "<div class='card mb-3'><div class='card-body'>";
			serverIndex += "IP address: <span class='float-right selectable'>" + WiFi.localIP().toString() + "</span><br/>";
			serverIndex += "Gateway address: <span class='float-right selectable'>" + WiFi.gatewayIP().toString() + "</span><br/>";
			serverIndex += "<form class='mb-0' method='POST' action='/net_cfg' enctype='multipart/form-data'>";
				serverIndex += "<div class='form-group'>";
					serverIndex += "<label for='hostnameInput' class='bmd-label-floating'>Hostname</label>";
					serverIndex += "<input name='hostname' type='text' class='form-control' id='hostnameInput' value='" + WiFi.hostname() + "' maxlength='32' >";
				serverIndex += "</div>";
				serverIndex += "<div class='form-group'>";
					serverIndex += "<label for='macAddrInput' class='bmd-label-floating'>MAC address</label>";
					serverIndex += "<input name='macaddr' type='text' class='form-control' id='macAddrInput' value='" + WiFi.macAddress() + "' maxlength='17' >";
				serverIndex += "</div>";
				serverIndex += "<button type='submit' class='btn btn-primary btn-raised'>Save</button>";
			serverIndex += "</form>";
		serverIndex += "</div></div></div>";
		
		serverIndex += "<div class='col-md'><h5><i class='material-icons'>system_update</i>&nbsp;Firmware update</h5>";
		serverIndex += "<div class='card mb-3'><div class='card-body'>";
			serverIndex += "<form class='mb-0' method='POST' action='/update' enctype='multipart/form-data'>";
				serverIndex += "<div class='form-group'>";
					serverIndex += "<input type='file' class='form-control-file' name='update'>";
				serverIndex += "</div>";
				serverIndex += "<button type='submit' class='btn btn-primary btn-raised'>Update</button>";
			serverIndex += "</form>";
		serverIndex += "</div></div>";
	serverIndex += "</div></div></div>";
	
	serverIndex += "</div>";
	serverIndex += "<script src='https://unpkg.com/popper.js@1.12.6/dist/umd/popper.js' integrity='sha384-fA23ZRQ3G/J53mElWqVJEGJzU0sTs+SvzG8fXVWP+kJQ1lwFAOkcUOysnlKJC33U' crossorigin='anonymous'></script>";
	serverIndex += "<script src='https://unpkg.com/bootstrap-material-design@4.1.1/dist/js/bootstrap-material-design.js' integrity='sha384-CauSuKpEqAFajSpkdjv3z9t8E7RlpJ1UP0lKM/+NdtSarroVKu069AlsRPKkFBz9' crossorigin='anonymous'></script>";
	serverIndex += "<script>$(function () { $('body').bootstrapMaterialDesign(); $('[data-toggle=\"tooltip\"]').tooltip(); });</script>";
	serverIndex += "</body>";
	serverIndex += "</html>";
	
	server.send(200, "text/html", serverIndex);
}
void handleUploadStart() {
	HTTPUpload& upload = server.upload();
	if(upload.status == UPLOAD_FILE_START) {
		Serial.setDebugOutput(true);
		Serial.printf("Update: %s\n", upload.filename.c_str());
		uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
		if(!Update.begin(maxSketchSpace)){//start with max available size
			Update.printError(Serial);
		}
	} else if(upload.status == UPLOAD_FILE_WRITE){
		if(Update.write(upload.buf, upload.currentSize) != upload.currentSize){
			Update.printError(Serial);
		}
	} else if(upload.status == UPLOAD_FILE_END){
		if(Update.end(true)){ //true to set the size to the current progress
			Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
		} else {
			Update.printError(Serial);
		}
		Serial.setDebugOutput(false);
	}
	yield();
}
void handleUploadEnd() {
	sendDeviceRestartPage(Update.hasError() ? "Update failed" : "Update success");
}


void wifiConnect() {
	WiFi.disconnect();
	Serial.println("Connecting to wifi...");
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	
	WiFi.setAutoConnect(true);
	WiFi.setAutoReconnect(true);
	WiFi.setSleepMode(WIFI_NONE_SLEEP);
	
	Serial.print("SLEEP MODE: ");
	switch (WiFi.getSleepMode()) {
		case WIFI_NONE_SLEEP:
			Serial.println("none");
			break;
		case WIFI_LIGHT_SLEEP:
			Serial.println("light");
			break;
		case WIFI_MODEM_SLEEP:
			Serial.println("modem");
			break;
	}
	WiFi.printDiag(Serial);
}
void WiFiEvent(WiFiEvent_t event) {
	Serial.printf("[WiFi-event] event: %d\n", event);
	switch(event) {
		case WIFI_EVENT_STAMODE_GOT_IP:
			Serial.println("WiFi connected");
			Serial.print("\tIP address: ");
			Serial.println(WiFi.localIP());
			Serial.print("\tgateway address: ");
			Serial.println(WiFi.gatewayIP());
			break;
		case WIFI_EVENT_STAMODE_DISCONNECTED:
			Serial.println("WiFi lost connection");
			passiveDelay(1000);
			wifiConnect();
			break;
		case WIFI_EVENT_STAMODE_CONNECTED:
		case WIFI_EVENT_STAMODE_AUTHMODE_CHANGE:
		case WIFI_EVENT_STAMODE_DHCP_TIMEOUT:
		case WIFI_EVENT_SOFTAPMODE_STACONNECTED:
		case WIFI_EVENT_SOFTAPMODE_STADISCONNECTED:
		case WIFI_EVENT_SOFTAPMODE_PROBEREQRECVED:
		case WIFI_EVENT_MODE_CHANGE:
		default:
		break;
	}
}


void ICACHE_RAM_ATTR ramMeasureTimerISR(){
	// calling any function not in IRAM might result in a reset (exception code 0 - "illegal instruction")
	// this occures when the chip is writing to FLASH when this ISR is called, as the instructions are unmaped
	// from CPU ram, so they cannot be read when in this ISR.
	freeRamCircBuffPtr++;
	
	freeRamRollingAvarageSum -= freeRamCircBuff[freeRamCircBuffPtr & MAX_SAMPLES];	// avg -= samples[T-NUM_SAMPLES] (which is samples[T] because of circular buffer)
	freeRamCircBuff[freeRamCircBuffPtr & MAX_SAMPLES] = system_get_free_heap_size();//ESP.getFreeHeap();			// samples[T] = curr value
    freeRamRollingAvarageSum += freeRamCircBuff[freeRamCircBuffPtr & MAX_SAMPLES];	// avg += curr value
}


void setup(void) {
	/*
	//timer1_disable();
	ETS_FRC_TIMER1_INTR_ATTACH(ramMeasureTimerISR, NULL); // timer1_isr_init();
    ETS_FRC1_INTR_ENABLE(); // timer1_attachInterrupt(ramMeasureTimerISR);
    T1C = (1 << TCTE) | ((TIM_DIV1 & 3) << TCPD) | ((TIM_EDGE & 1) << TCIT) | ((TIM_LOOP & 1) << TCAR);
	T1I = 0;			//timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
	
    T1L = ((625000)& 0x7FFFFF);
    if ((T1C & (1 << TCIT)) == 0) TEIE |= TEIE1;//edge int enable //timer1_write(625000); 
	delay(25);
	*/
	
	// timer1_isr_init();
	// timer1_attachInterrupt(ramMeasureTimerISR);
	// timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP); // might need TIM_LEVEL see https://bbs.espressif.com/viewtopic.php?t=2630 also https://www.esp8266.com/viewtopic.php?f=4&t=10600&start=8
	// When I "or" 0x01 into the value that is written into FRC1_CTRL_ADDRESS (+ PERIPHS_TIMER_BASEADDR of course), it works.
	// According to the "documentation" (ehhhrmm, just examples found here and there), LEVEL_INT would be 0x1 and EDGE_INT would be 0x0. It seems it's the other way around actually?
	// It IS an EDGE interrupt now, because I don't need to clear the interrupt mask to have the ISR called again.
	// ((1s / NUM_SAMPLES) * 1000000) = 7812.5us
	// maximum ticks 8388607
	// (80/TIM_DIV1)*7812.5 = 625000
	// (clockCyclesPerMicrosecond() / 1) * 7812.5 // clockCyclesPerMicrosecond() macro might not work is CPU clock is changed
	// timer1_write(625000);
	
	
	//system_update_cpu_freq(160);
	pinMode(LED_PIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
	digitalWrite(LED_PIN, 0); // startup as on
	// TODO: add pulldown resistor, so lamp doesn't flash @ startup when uC pin is set as input
	
	// analogWriteFreq(50); // TODO: for pwm to work, we need a zero-cross detector
	
	
	EEPROM.begin(EEPROM_SIZE);
	if (EEPROM_read_uint8(EEPROM_ADDR_1_MEM_COHERENCY_FLAG) != EEPROM_VAL_MEM_COHERENCY_FLAG) {
		String defaultHostname = "ESP_" + String(random(1000, 9999));
		const unsigned char defaultMAC[] = {0x5C, 0xCF, 0x7F, 0x96, (const unsigned char)random(0, 255), (const unsigned char)random(0, 255)};
		
		EEPROM_write_uint8(EEPROM_ADDR_1_MEM_COHERENCY_FLAG, EEPROM_VAL_MEM_COHERENCY_FLAG);
		EEPROM_write_uint8(EEPROM_ADDR_1_SET_INTENSITY_ON_BOOT, 0);
		EEPROM_write_uint8_arr(EEPROM_ADDR_33_HOSTNAME, defaultHostname.c_str(), 33);
		EEPROM_write_uint8_arr(EEPROM_ADDR_6_MAC_ADDR, (const char*)defaultMAC, 6);
		EEPROM.commit();
	}
	
	
	
	setIntensityOnBoot = EEPROM_read_uint8(EEPROM_ADDR_1_SET_INTENSITY_ON_BOOT);
	if (setIntensityOnBoot) {
		setLedIntensity(EEPROM_read_uint32(EEPROM_ADDR_4_LED_INTENSITY));
	} else {
		setLedIntensity(0); // default setting is on
	}
	
	char hostname[33];
	unsigned char mac[6];
	
	EEPROM_read_uint8_arr(EEPROM_ADDR_33_HOSTNAME, hostname, 33);
	EEPROM_read_uint8_arr(EEPROM_ADDR_6_MAC_ADDR, (char *)mac, 6);
	
	if (!wifi_set_macaddr(STATION_IF, mac)) {} // TODO: how to handle error here?
	WiFi.hostname(hostname);
	
	
	
	
	
	Serial.begin(115200);
	Serial.println();
	Serial.println("Booting Sketch...");
	
	WiFi.onEvent(WiFiEvent);
	wifiConnect();
	
	if (WiFi.waitForConnectResult() == WL_CONNECTED) {
		server.on("/",			HTTP_GET,	handleMain);
		server.on("/info",		HTTP_POST,	handleInfo);
		server.on("/intensity",	HTTP_POST,	handleIntensity);
		server.on("/toggle",	HTTP_POST,	handleToggle);
		server.on("/config",	HTTP_POST,	handleConfig);
		server.on("/net_cfg",	HTTP_POST,	handleNetConfig);
		server.on("/update",	HTTP_POST,	handleUploadEnd, handleUploadStart);
		server.onNotFound ( handleNotFound );
		server.begin();
		
		Serial.printf("Ready! Open in your browser\n");
	} else {
		Serial.println("WiFi Failed");
	}
}


void loop(void) {
	unsigned long deltaTime = micros() - lastAggregateTime;
	if (deltaTime > aggregateInterval) {
		lastAggregateTime = micros();
		
		userTime = deltaTime - OStimeAcc;
		OStime = OStimeAcc;
		OStimeAcc = 0;
	}
	
	server.handleClient();
	delay(1); // give the web browser time to receive the data
	
	if (Serial.available()) {
		Serial.write("Pong: ");
		Serial.write(Serial.read());
		Serial.write("\n");
	}
	if (millis() - lastPingTime > 10000) {
		millis64(); // call at regular intervals, so we don't loose track of rollovers (every 70min?)
		lastPingTime = millis();
		Serial.print("Ping");
		
		if(ping_client.connect(WiFi.gatewayIP().toString().c_str(), 80)) { // TODO: port 80 isn't always open for example on https router authentication
			Serial.println(" OK");
			ping_client.stop();
		} else {
			Serial.println("Ping FAIL");
			wifiConnect();
			// TODO: before reconnect, check connection status, don't reconnect, if already connecting or connected
			// TODO: test
			// WiFi.reconnect();
		}
	}
	
	yield();
}



