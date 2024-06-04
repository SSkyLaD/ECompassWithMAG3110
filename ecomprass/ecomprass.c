#include "board.h"
#include "fsl_i2c.h"
#include <math.h>
#include "fsl_port.h"
#include "MKL46Z4.h"
#include "stdio.h"
#include "slcd.h"


#define MAG_I2C_CLK_FREQ CLOCK_GetFreq(I2C0_CLK_SRC)
#define I2C_BAUDRATE 100000U
#define MAG3110_CTRL_REG1 0x10U

static bool I2C_WriteMagReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value);
static bool I2C_ReadMagRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize);

/* MAG3110 device address */
const uint8_t g_mag_address[] = {0x0EU};

i2c_master_handle_t g_m_handle;

uint8_t g_mag_addr_found = 0x00;

volatile bool completionFlag = false;
volatile bool nakFlag = false;

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    /* Signal transfer success when received success status. */
    if (status == kStatus_Success)
    {
        completionFlag = true;
    }
    /* Signal transfer success when received success status. */
    if ((status == kStatus_I2C_Nak) || (status == kStatus_I2C_Addr_Nak))
    {
        nakFlag = true;
    }
}

void init_i2c()
{
	uint32_t sourceClock = MAG_I2C_CLK_FREQ;
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Bps = I2C_BAUDRATE;
    I2C_MasterInit(BOARD_ACCEL_I2C_BASEADDR, &masterConfig, sourceClock);

}
static bool I2C_WriteMagReg(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t value)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &value;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status_t result = I2C_MasterTransferBlocking(base, &masterXfer);
    if (result == kStatus_Success)
    {
        return true;
    }
    else
    {
        return false;
    }
}

static bool I2C_ReadMagRegs(I2C_Type *base, uint8_t device_addr, uint8_t reg_addr, uint8_t *rxBuff, uint32_t rxSize)
{
    i2c_master_transfer_t masterXfer;
    memset(&masterXfer, 0, sizeof(masterXfer));

    masterXfer.slaveAddress = device_addr;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = reg_addr;
    masterXfer.subaddressSize = 1;
    masterXfer.data = rxBuff;
    masterXfer.dataSize = rxSize;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    status_t result = I2C_MasterTransferBlocking(base, &masterXfer);
    if (result == kStatus_Success)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int16_t heading;
bool state = false;
bool set = false;
void initLed(void);
void init_SysTick_interrupt(void);
void SysTick_Handler(void);
void blinkLedRed(void);
void blinkLedGreen(void);
void initSwitch(void);
void PORTC_PORTD_IRQHandler(void) ;
int16_t calculateHeading(int16_t x, int16_t y, int16_t xMax, int16_t yMax, int16_t xMin, int16_t yMin); 

int16_t calculateHeading(int16_t x, int16_t y, int16_t xMax, int16_t yMax, int16_t xMin, int16_t yMin) {
		int16_t xoffset = (xMax + xMin) / 2;
		int16_t yoffset = (yMax + yMin) / 2;
	
		float scalex = (float) 2 / (xMax - xMin);
		float scaley = (float) 2 / (yMax - yMin);
	
		
		float x1 = (float) (x - xoffset) * scalex;
		float y1 = (float) (y - yoffset) * scaley;
    // Calculate the heading using the atan2 function
    float heading_rad = atan2f(y1, x1);
    int16_t heading_deg = (int16_t)(heading_rad * 180.0 / 3.1415);

    // Adjust the heading to ensure it's within the range [0, 360)
    if (heading_deg < 0) {
        heading_deg += 360;
    }
		if (heading_deg > 360) {
				heading_deg -= 360;
		}

    return heading_deg;
}

void displayHeading(int heading) {
    char buffer[20];  
    sprintf(buffer, "%d", heading); 
		
    SLCD_WriteMsg((unsigned char *)buffer);  
}

void initLed(void) {
    SIM->SCGC5 |= (1 << 13) | (1 << 12); 
    PORTE->PCR[29] = (1 << 8); 
		PORTD->PCR[5] = (1 << 8);
    PTE->PDDR |= (1 << 29);
		PTD->PDDR |= (1 << 5);
}

int32_t volatile msTicks = 0; // Interval counter in ms
void init_SysTick_interrupt()
{
	SysTick->LOAD = SystemCoreClock / 1000 - 1; 
	SysTick->VAL = 0;
	SysTick->CTRL = (1<<0) | (1<<1) |(1<<2);
}

void SysTick_Handler (void) { // SysTick interrupt Handler
	msTicks++;
}

void Delay(uint32_t TICK) {
    while (msTicks < TICK);
}

void blinkLedRed(void)
{
	uint32_t i = 0;
	PTE -> PCOR = (1<<29);
	Delay(249);
	PTE -> PSOR = (1<<29);
	Delay(249);	
}

void blinkLedGreen(void)
{
	uint32_t i = 0;
	PTD -> PCOR = (1<<5);
	Delay(499); 
	PTD -> PSOR = (1<<5);
	Delay(499);	
}

void initSwitch(void) 
{
	SIM->SCGC5 |= (1 << 11);
	PORTC->PCR[3] = (1 << 8)  | (1 << 0) | (1 << 1);
	PTC->PDDR &= ~((uint32_t)(1u << 3));
	PORTC->PCR[3] |= (1 << 17) | (1 << 19);
	
	PORTC->PCR[12] = (1 << 8) | (1 << 0) | (1 << 1); 
  PTC->PDDR &= ~((uint32_t)(1u << 12));           
  PORTC->PCR[12] |= (1 << 17) | (1 << 19); 
	NVIC_ClearPendingIRQ(31);
	NVIC_EnableIRQ(31);
}

void initNVIC(void) {
    NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);
    NVIC_EnableIRQ(PORTC_PORTD_IRQn);
    NVIC_SetPriority(PORTC_PORTD_IRQn, 2);
}

void PORTC_PORTD_IRQHandler(void) 
{ 
		uint32_t i = 0;
		for (i = 0; i < 1000; i++);
	
		if ((PTC -> PDIR & (1 << 12)) == 0) 	
		{
			PORTC->ISFR = (1 << 12);
			PTE -> PSOR = (1<<29);
			PTD -> PSOR = (1<<5);
			SLCD_WriteMsg((unsigned char *)"   ");
			while(1);
		}
		
		if ((PTC -> PDIR & (1 << 3)) == 0)
		{
			PORTC->ISFR = (1 << 3);
			state = !state;
		}
		
}

// ham hieu chinh cam bien tu truong
void calibration(int16_t *xMax, int16_t *yMax, int16_t *xMin, int16_t *yMin)
{
	SLCD_WriteMsg((unsigned char *) "turn");
        uint8_t databyte = 0;
        uint8_t write_reg = 0;
        uint8_t readBuff[6];
        int16_t x, y, z;
        uint32_t i = 0U;

        write_reg = MAG3110_CTRL_REG1;
        databyte = 0x01; // Active mode
        I2C_WriteMagReg(I2C0, 0x0E, write_reg, databyte);
        
        for (int i = 0; i <300; i++) {
						I2C_ReadMagRegs(I2C0, 0xE, 0x01, readBuff, 6);

						x = (int16_t)((readBuff[0] << 8) | readBuff[1]);
						y = (int16_t)((readBuff[2] << 8) | readBuff[3]);
						z = (int16_t)((readBuff[4] << 8) | readBuff[5]);
					
						if (x > *xMax) *xMax = x;
						if (x < *xMin) *xMin = x;
						if (y > *yMax) *yMax = y;
						if (y < *yMin) *yMin = y;
						Delay(50);
				}
				SLCD_WriteMsg((unsigned char *) "done");	

}	

int main(void)
{
    bool isThereMag = false;
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_I2C_ConfigurePins();
		SLCD_Init();
		initLed();
		initSwitch();
		initNVIC(); 
		init_SysTick_interrupt();
		init_i2c();
		blinkLedRed();
		int16_t xMax = -10000;
		int16_t xMin = 10000;
		int16_t yMax = -10000;
		int16_t yMin = 10000;
		calibration(&xMax, &yMax, &xMin, &yMin);
    
		
    while (1) {
			uint8_t databyte = 0;
        uint8_t write_reg = 0;
        uint8_t readBuff[6];
        int16_t x, y, z;
        uint32_t i = 0U;

        write_reg = MAG3110_CTRL_REG1;
        databyte = 0x01; // Active mode
        I2C_WriteMagReg(I2C0, 0x0E, write_reg, databyte);

        I2C_ReadMagRegs(I2C0, 0xE, 0x01, readBuff, 6);

            x = (int16_t)((readBuff[0] << 8) | readBuff[1]);
            y = (int16_t)((readBuff[2] << 8) | readBuff[3]);
            z = (int16_t)((readBuff[4] << 8) | readBuff[5]);
					
						heading = calculateHeading(x, y, xMax, yMax, xMin, yMin);
						
						if (state == true) // state on thi bat dau
							{
								PTE -> PSOR = (1<<29);
								blinkLedGreen();
								displayHeading(heading);
							} else  // state off thi tam dung
							{
								PTD->PSOR = (1 << 5);
								blinkLedRed();
							}
		}
}
