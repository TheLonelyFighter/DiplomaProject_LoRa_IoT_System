/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * Acest cod a fost modificat pentru a putea transmite temperatura folosind 
 * senzorul internn de temperatura din microcontroller-ul AtMega328, de pe 
 * o placuta Arduino NANO conectata la un modul de transmisiune LORA
 *
 * Codul foloseste metoda de activare ABP (Activation-by-Personalization)
 *******************************************************************************/

#include <lmic.h> 
#include <hal/hal.h>
#include <SPI.h>

// Cheia sesiunii de retea generata de consola TTN la inregistrarea dispozitivului
static const PROGMEM u1_t NWKSKEY[16] = { 0x24, 0x32, 0x93, 0x5D, 0xCF, 0x27, 0x25, 0xDA, 0x87, 0x44, 0x35, 0x8D, 0x9A, 0x42, 0xAD, 0xAB };
                                          
// Cheia sesiunii de aplicatie generata de consola TTN la inregistrarea dispozitivului
static const u1_t PROGMEM APPSKEY[16] = { 0xDB, 0xEF, 0xF3, 0xE3, 0xC4, 0x49, 0x3C, 0xA6, 0xC8, 0x98, 0x50, 0xA8, 0xE3, 0xC0, 0xFB, 0xCF };


// Adresa unica a dispozitivului, utilizata de gateway pentru a transmite datele primite de nod catre serverul aplicatiei
static const u4_t DEVADDR = 0x26011A32 ; 

// Prototipuri de functie utilizate in cazul activarii OTAA
// Nu pot fi sterse complet deoarece acest lucru genereaza erori
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;
uint8_t temperature[]= "20.20"; // vector utlizat pentu transmiterea temperaturii cu functia LMIC_setTxData2()

// Intervalul de timp in secunde la care este transmis un pachet
const unsigned TX_INTERVAL = 60;



// Conexiunea pinilor depinde de modul de conectare dintre Arduino Nano si modului de transmisiune LoRa
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {6, 5, 4},
};

int channel = 0; //canal corespunzator frecventei 868.1 Mhz
int dr = DR_SF7; //Factor de imprastiere SF 7

//Functii citire temperatura

long read_temp()
{
  // Citeste temperatura senzorului 
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  // Conversie analog-digitala
  ADCSRA |= _BV(ADEN) | _BV(ADSC);
  // Detecteaza sfarsitul conversiei
  while (bit_is_set(ADCSRA,ADSC));
  // Returneaza valoarea necalibrata a temperaturii
  return ADCL | (ADCH << 8);
}
 
// Converteste temperatura citita in format virgula mobila
double conv_temp(long raw_temp)
{
  return((raw_temp - 324.31) / 1.22); //coeficienti de calibrare calculati de producatorul
									 //microconttroler-ului
}

// Dezactiveaza toate canalele in afara de cel folosit
// deoarece gateway-ul utilizeaza un singur canal
void forceTxSingleChannelDr() {
    for(int i=0; i<9; i++) { 
        if(i != channel) {
            LMIC_disableChannel(i);
        }
    }
    // Seteaza factorul de imprastiere folosit si puterea de transmisiune in mW
    LMIC_setDrTxpow(dr, 14);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Programeaza urmatoarea transmsiune
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // Slot de asteptare pentru Clasa A de dispozitive pentru mesaje pe legatura descendenta
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    double sensorData = conv_temp(read_temp()); //citeste temperatura senzorului
    Serial.println(sensorData, 2);
    char TempString[6];  // variabila auxiliara pentru conversie float -> char
    dtostrf(sensorData,2,2,TempString); //conversie temperatura float -> char
    Serial.println(TempString);
    temperature[0] = (unsigned char) TempString[0]; //conversie temperatura char -> unsigned char
    temperature[1] = (unsigned char) TempString[1];
    temperature[2] = (unsigned char) TempString[2];
    temperature[3] = (unsigned char) TempString[3];
    temperature[4] = (unsigned char) TempString[4];
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Pregateste transmisiunea pe cale ascendenta
        LMIC_setTxData2(1, temperature, sizeof(temperature)-1, 0);
        Serial.println(F("Packet queued"));
    }   
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    #ifdef VCC_ENABLE
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // Functii de initializare din biblioteca lmic.h
    os_init();
    LMIC_reset();

    // Seteaza cheile folosite in sesiune de transmisie
    #ifdef PROGMEM
    // In cazul AVR, aceste valori trebuie tranferate din memoria
	// flash in memoria RAM 
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else    
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Configureaza cele 8 canale de frecventa folosite de LoRa in Europa
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      
    #elif defined(CFG_us915)   
    LMIC_selectSubBand(1);
    #endif

    LMIC_setLinkCheckMode(0);

    // Calea descendenta foloseste SF = 9
    LMIC.dn2Dr = DR_SF9;

    //Foloseste un singur canal datorita gateway-ului
    forceTxSingleChannelDr();
	
	//Eroare de sincronizare permisa in determinarea ferestrei RX2
    LMIC_setClockError (MAX_CLOCK_ERROR * 10/100);  
    
    // Trimite pachetul pe calea ascendenta
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
