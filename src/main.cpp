#include <Arduino.h>
#include <PubSubClient.h>
#include <Ethernet.h>
#include <SPI.h>
#include <string.h>

//defines
#define LIN_BAUDRATE 19200 // Example LIN bus baud rate

// variables for the program
volatile bool timerFlag = false;
unsigned int cnt=0;

// Ethernet parameters
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }; // MAC address for the Ethernet shield
IPAddress ip(192, 168, 1, 100);  // IP address of the Ethernet shield
IPAddress server(192, 168, 1, 2); // IP address of the MQTT broker

// MQTT parameters
const char* mqtt_server = "192.168.1.2"; // MQTT broker IP address
const int mqtt_port = 1883;
const char* mqtt_user = "your_username"; // MQTT username
const char* mqtt_pass = "your_password"; // MQTT password
const char* subscribe_topic = "inTopic"; // Topic to subscribe to
const char* publish_topic = "outTopic";  // Topic to publish to

EthernetClient ethClient;
PubSubClient mqttClient(ethClient);

//Functions prototypes
void configure_gclk(void);
void USART_lin_setup(void);
void lin_switch_rx_tx(String mode);
void send_lin_header(uint8_t id);
void send_lin_frame(uint8_t id, uint8_t* data, uint8_t length);
void receive_lin_frame(uint8_t* id, uint8_t* data, uint8_t length);

//Configure clock generator 0 with DFLL48M as source
void configure_gclk(void) {
    SYSCTRL->DFLLCTRL.reg = SYSCTRL_DFLLCTRL_ENABLE;
    while (!(SYSCTRL->PCLKSR.reg & SYSCTRL_PCLKSR_DFLLRDY));

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(0) |
                        GCLK_GENCTRL_SRC_DFLL48M |
                        GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

    GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(1) |
                        GCLK_GENCTRL_SRC_DFLL48M |
                        GCLK_GENCTRL_GENEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}

void configureExternalInterrupt() {
    // Enable the APB clock for EIC
    PM->APBAMASK.reg |= PM_APBAMASK_EIC;

    // Configure the Generic Clock for EIC
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_EIC | // Select EIC
                        GCLK_CLKCTRL_GEN_GCLK0 | // Use GCLK0 (DFLL48M)
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY); // Wait for synchronization

    // Configure PB22 as an external interrupt pin
    PORT->Group[1].PINCFG[22].bit.PMUXEN = 1; // Enable peripheral multiplexing on PB22
    PORT->Group[1].PMUX[22 >> 1].reg |= PORT_PMUX_PMUXE_A; // Peripheral function A for EIC on PB22

    // Disable the EIC before configuration
    EIC->CTRL.reg &= ~EIC_CTRL_ENABLE;
    while (EIC->STATUS.reg & EIC_STATUS_SYNCBUSY);

    // Configure the EIC to use edge detection on PB22 (EXTINT[6] is PB22)
    EIC->CONFIG[2].reg |= EIC_CONFIG_SENSE6_FALL; // Detect falling edge
    EIC->INTENSET.reg = EIC_INTENSET_EXTINT6; // Enable interrupt on EXTINT[6]

	// Enable EIC
	EIC->CTRL.reg |= EIC_CTRL_ENABLE;
	while (EIC->STATUS.reg & EIC_STATUS_SYNCBUSY);

	// Enable the interrupt in NVIC
	NVIC_EnableIRQ(EIC_IRQn);
}

void timer_TC3_setup() {
	// https://developerhelp.microchip.com/xwiki/bin/view/products/mcu-mpu/32bit-mcu/sam/samd21-mcu-overview/samd21-processor-overview/samd21-nvic-overview/code-gcc-nvic-example/
	// Set up Timer/Counter
	// Source clock config  ----->  https://forum.arduino.cc/t/generating-varying-frequencies-on-pin-pa21-for-atsamd21g18au/897380/4
	// Configure synchronous bus clock
	PM->APBCMASK.reg |= PM_APBCMASK_TC3;			// enable TC3 interface

	// Enable and configure GCLK for TC3 (Timer/Counter 3)
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_TCC2_TC3 |	 // Select tcc2 and tc3
						GCLK_CLKCTRL_GEN_GCLK0 |     // Use GCLK0 (DFLL48M)
						GCLK_CLKCTRL_CLKEN ;	 	// enable 
	
	// Reset TC3
    TC3->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
    while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); // Wait for synchronization

	// Set Timer Mode to 16-bit
	TC3->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16 |		// Set Timer Mode to 16-bit
							 TC_CTRLA_PRESCALER_DIV1024 |	// Set prescaler to 1024 (approx. 100ms with 48MHz clock)
							 TC_CTRLA_WAVEGEN_MFRQ ;		// Configure TC3 Compare Mode for compare channel 0, "Match Frequency" operation

	// Set timer compare value for 100ms
	uint32_t compare_value = ( 48000000 / 1024 ) * 0.1;
	TC3->COUNT16.CC[0].reg = (uint16_t)compare_value ;
	while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

	// Enable compare interrupt
	TC3->COUNT16.INTENSET.reg = TC_INTENSET_MC0;

	// Enable timer TC3
	TC3->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	while (TC3->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);

	// Enable NVIC interrupt TC3
	NVIC_EnableIRQ(TC3_IRQn);
}

void USART_lin_setup() {
	PM->APBCMASK.reg = PM_APBCMASK_SERCOM1 ;	// Enable SERCOM1 APB

	// Configure the Generic Clock 0 for SERCOM1
    GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM1_CORE | // Select SERCOM1 core
                        GCLK_CLKCTRL_GEN_GCLK0 | // Use GCLK0 (DFLL48M)
                        GCLK_CLKCTRL_CLKEN;
    while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY); // Wait for synchronization

	// Configure PA18 for SERCOM1 (both RX and TX will use this pin)
    PORT->Group[0].PINCFG[18].bit.PMUXEN = 1;
    PORT->Group[0].PMUX[18 >> 1].bit.PMUXE = PORT_PMUX_PMUXE_C; // Peripheral function C for SERCOM1 on PA18

	// Configure SERCOM1 for USART in LIN mode
    SERCOM1->USART.CTRLA.reg = SERCOM_USART_CTRLA_DORD | // LSB first
                               SERCOM_USART_CTRLA_MODE_USART_INT_CLK | // Internal clock
                               SERCOM_USART_CTRLA_RXPO(2) | // RX on PAD[2] (PA18)
                               SERCOM_USART_CTRLA_TXPO(2) | // TX on PAD[2] (PA18)
                               SERCOM_USART_CTRLA_FORM(0x4); // LIN Mode


	// frequency used may have to be changed because of baudrate 
	// Set the baud rate for LIN (typically 19200 bps)
    uint16_t baud = 65536 - (1- ( 16 * (LIN_BAUDRATE / 48000000)));
    SERCOM1->USART.BAUD.reg =(uint16_t) baud;

    // Enable the receiver and transmitter
    SERCOM1->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN |
							   SERCOM_USART_CTRLB_TXEN;
   
    while (SERCOM1->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_CTRLB);

    // Enable SERCOM1
    SERCOM1->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
    while (SERCOM1->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_ENABLE);
}

// Function to switch between RX and TX modes
void lin_switch_rx_tx(String mode) {
	if(mode=="RX" || mode=="rx"){
		// Disable Transmitter
        SERCOM1->USART.CTRLB.reg &= ~SERCOM_USART_CTRLB_TXEN;
        while (SERCOM1->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_CTRLB);

        // Configure PA18 as SERCOM1 PAD[2] (RX)
        PORT->Group[0].DIRCLR.reg = PORT_PA18; // Set PA18 as input
		
		// Enable Receiver
        SERCOM1->USART.CTRLB.reg |= SERCOM_USART_CTRLB_RXEN; 
        while (SERCOM1->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_CTRLB);

        SerialUSB.println("LIN cable as Receiver");
	}else if (mode=="TX" || mode=="tx"){
		// Disable Receiver
        SERCOM1->USART.CTRLB.reg &= ~SERCOM_USART_CTRLB_RXEN;
        while (SERCOM1->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_CTRLB);

        // Configure PA18 as SERCOM1 PAD[2] (TX)
        PORT->Group[0].DIRSET.reg = PORT_PA18; // Set PA18 as output

		// Enable Transmitter
        SERCOM1->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN; 
        while (SERCOM1->USART.SYNCBUSY.reg & SERCOM_USART_SYNCBUSY_CTRLB);

        SerialUSB.println("LIN cable as Transmitter");
	}
}

void send_lin_header(uint8_t id) {
    uint8_t sync_byte = 0x55;

    // Configure pin as TX
    lin_switch_rx_tx("TX");

    // Save current baud rate
    uint16_t original_baud = SERCOM1->USART.BAUD.reg;

    // Set a very low baud rate for the break signal (e.g., 9600 / 13)
    SERCOM1->USART.BAUD.reg = 0xFFFF; // Set baud rate to minimum

    // Send break signal (long low signal)
    SERCOM1->USART.DATA.reg = 0x00; // Send break
    while (!(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC)); // Wait for break completion

    // Restore original baud rate
    SERCOM1->USART.BAUD.reg = original_baud;
  
    // Send sync byte
    SERCOM1->USART.DATA.reg = sync_byte;
    while (!(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC));

    // Send identifier
    SERCOM1->USART.DATA.reg = id;
    while (!(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC));
}

void send_lin_frame(uint8_t id, uint8_t* data, uint8_t length) {
    uint8_t checksum = 0;

    // Configure pin as TX
    lin_switch_rx_tx("TX");

    send_lin_header(id);

    // Send data and calculate checksum
    for (uint8_t i = 0; i < length; i++) {
        SERCOM1->USART.DATA.reg = data[i];
        while (!(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC));
        checksum += data[i];
        checksum = (checksum & 0xFF) + (checksum >> 8); // Carry add
    }

	checksum = ~checksum; // Invert the checksum

    // Send checksum
    SERCOM1->USART.DATA.reg = checksum;
    while (!(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_TXC)); // Wait for transmission complete
}

void receive_lin_frame(uint8_t* id, uint8_t* data, uint8_t length) {
    int checksum = 0;
	uint8_t received_checksum;

    // Configure pin as RX
    lin_switch_rx_tx("RX");

	// Receive identifier
    while (!(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC));
    *id = SERCOM1->USART.DATA.reg;
    checksum += *id;

    // Receive data bytes and calculate checksum
    for (uint8_t i = 0; i < length; i++) {
        while (!(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC));
        data[i] = SERCOM1->USART.DATA.reg;
        checksum += data[i];
		checksum = (checksum & 0xFF) + (checksum >> 8); // Carry add
    }

    // Receive and verify checksum
    while (!(SERCOM1->USART.INTFLAG.reg & SERCOM_USART_INTFLAG_RXC));
    received_checksum = SERCOM1->USART.DATA.reg;
    checksum = ~checksum;

    if (received_checksum != checksum) {
        // Handle checksum error
        SerialUSB.println("Checksum error!");
    }
}

void lin_sendprint_every_id(uint8_t* buffer, uint8_t length){
	uint8_t *id = 0x00;

	for ( uint8_t i = 0x00; i < 0x3F ; i++) {
		id++;
		send_lin_header(*id);
		receive_lin_frame(id, buffer, length);
		SerialUSB.print("ID: ");
		SerialUSB.print(*id, HEX);
		SerialUSB.print(" | Data ->");
		for (uint8_t j = 0; j< length ; j++) {
			SerialUSB.print(buffer[j]);
			SerialUSB.print(".");
		}
		SerialUSB.println();
	}
}

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttClient.connect("SAMD21Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      // Subscribe to the topic
      mqttClient.subscribe(subscribe_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

// Function to publish a message to a topic
void publishMessage(const char* topic, const char* message) {
  if (mqttClient.connected()) {
    mqttClient.publish(topic, message);
    Serial.print("Published message to ");
    Serial.print(topic);
    Serial.print(": ");
    Serial.println(message);
  } else {
    Serial.println("MQTT client not connected.");
  }
}

void turn_on_off_boat(String mode)
{
	if(mode=="ON" || mode == "on")
	{
		//turn on relé pin x
	}else if (mode == "OFF" || mode == "off")
	{
		//turn on relé pin x
	}
	
}

void setup()
{
    Serial.begin(9600);
	configure_gclk();
	USART_lin_setup();
	timer_TC3_setup();
	configureExternalInterrupt();

    /*
    //mqtt setup
    Ethernet.begin(mac, ip); // Start Ethernet with the MAC address specified
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(callback);
    // Ensure we are connected
    reconnect();
    */
}

void loop()
{
	uint8_t* data={};
	uint8_t length=8;
	lin_sendprint_every_id(data, length);
    /*
    if (!mqttClient.connected()) {
        reconnect();
    }
    publishMessage(const char *topic, const char *message);
    mqttClient.loop();
    */
}


// Timer/Counter 3 IRQ Handler
void TC3_Handler() {
  // Check if the interrupt is caused by compare match channel 0
  if (TC3->COUNT16.INTFLAG.bit.MC0){
	// Clear the interrupt flag
	TC3->COUNT16.INTFLAG.bit.MC0 = 1;
    cnt++;
    timerFlag = false;
    if(cnt==100){
    	SerialUSB.println("10 segundos");
    	cnt=0;
    	timerFlag = true;
    }
  }
}
void EIC_Handler(void) {
    // Clear the interrupt flag
	EIC->INTFLAG.reg = (1<<EXTERNAL_INT_6);

    // Your interrupt handling code here
}