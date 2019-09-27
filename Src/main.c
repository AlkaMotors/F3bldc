/** Test Program for STM32F3 esc version 2, with Drv8323 driver.
 * Thanks to o-drive for SPI code for TI driver setup https://github.com/madcowswe/ODrive
 *
 * TODO, work on startup sequence. currently might not start up all motors.
 * possibly polling comparator at startup to limit triggers.
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#define ADR_FAULT_STAT      (0x00)
#define ADR_VGS_STAT        (0x01)
#define ADR_DRV_CTRL        (0x02)
#define ADR_GATE_DRV_HS     (0x03)
#define ADR_GATE_DRV_LS     (0x04)
#define ADR_OCP_CTRL        (0x05)
#define ADR_CSA_CTRL        (0x06)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
COMP_HandleTypeDef hcomp1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;

/* USER CODE BEGIN PV */
int brake = 1;                   // turns on motor brake on 0 input , shorts motor windings
char slow_decay = 1;              // turns on complementary PWM with active braking 1 or freewheel 0
int start_power = 325;            // startup duty cycle , out of 1499
char advancedivisorup = 10;        // amount of advance when BEMF rising, 60 degree time / 10 = 6 degree advance for example
char advancedivisordown = 10;      // 60 degree time / advancedivisor = advance, should set proportianal to rpm in future
int pwm_settle = 60;             // blanking source duration for comparator, out of 1499
int start_com_interval = 10000;    // set a start interval after start sequence, small motors may need lower interval
int startupcycles = 15000;           // number of cycles to keep startup power reduction

int bi_direction = 0;
int max_servo_deviation = 150;

int bemf_counts;
int compit;
int filter_delay;
int filter_level;
char error = 0;
int signal_timeout_threshold = 20000;
int signaltimeout = 0;
int servoraw;
int adjusted_input = 0;
int commutate_count= 0;


int startup_throttle_limit_countdown;
int step = 1;
int timestamp;
int count = 10;
int upthiszctime = 0;
int uplastzctime = 0;
int startupcountdown=0;
char advancedivisor = 5;                    // increase divisor to decrease advance,
int thiszctime = 0;
int lastzctime = 0;
int sensorless = 0;
int commutation_interval = 0;
int advance = 0; // set proportianal to commutation time. with advance divisor
int blanktime = 2000;
int waitTime = 0;
int inputcapture = 0;
int zctimeout = 0;
int zc_timeout_threshold = 10000;   // depends on commutation speed, set in main loop
int duty_cycle = 0;
int pwm = 1;
int floating = 2;
int lowside = 3;
int forward = 1;
int rising = 1;
int running = 0;
int started = 0;
char armed = 0;
int armedcount = 0;
int input_buffer_size = 64;
int smallestnumber = 20000;
uint32_t dma_buffer[64];
int input = 0;
int newinput =0;
int voltageraw = 0;
int currentraw = 0;
uint32_t ADC1ConvertedValues[2] = {0,0};
char servoPwm = 0;
char inputSet = 0;


/* Private variables ---------------------------------------------------------*/
/*Register setup for the DRV8323 driver*/
uint16_t DRV8323regDrvCtrl =
  0 << 9  | //DIS_CPUV
  0 << 8  | //DIS_GDF
  0 << 7  | //OTW_REP
  0 << 5  | //PWM_MODE
  0 << 4  | //1PWM_COM
  0 << 3  | //1PWM_DIR
  0 << 2  | //COAST
  0 << 1  | //BRAKE
  0;        //CLR_FLT

uint16_t DRV8323regGateDrvHS =
  3 << 8  | //LOCK
  6 << 4 | //IDRIVEP_HS
  6;       //IDRIVEN_HS

uint16_t DRV8323regGateDrvLS =
  1 << 10 | //CBC
  2 << 8  | //TDRIVE
  6 << 4 | //IDRIVEP_LS                       // set very low for now
  6;       //IDRIVEN_LS

uint16_t DRV8323regOcpCtrl =
  1 << 10 | //TRETRY
  2 << 8  | //DEAD_TIME
  1 << 6  | //OCP_MODE
  1 << 4  | //OCP_DEG
  0;        //VDS_LVL

uint16_t DRV8323regCsaCtrl =
  0 << 10 | //CSA_FET
  1 << 9  | //VREF_DIV
  0 << 8  | //LS_REF
  2 << 6  | //CSA_GAIN
  0 << 5  | //DIS_SEN
  0 << 4  | //CSA_CAL_A
  0 << 3  | //CCSA_CAL_B
  0 << 2  | //CCSA_CAL_C
  3;        //CSEN_LVL
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_COMP1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM16_Init(void);
static void MX_SPI3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void DRV8323_writeSpi(uint8_t regAdr, uint16_t regVal)
{
	uint16_t controlword = (regAdr & 0x7) << 11 | (regVal & 0x7ff); //MSbit =0 for write, address is 3 bits (MSbit is always 0), data is 11 bits

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t*)(&controlword), 1, 1000);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	return;
}

uint16_t DRV8323_readSpi(uint8_t regAdr)
{
 //   uint16_t zerobuff = 0;
	uint16_t controlword = 0x8000 | (regAdr & 0x7) << 11; //MSbit =1 for read, address is 3 bits (MSbit is always 0), data is 11 bits
	uint16_t recbuff = 0xbeef;

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)(&controlword), (uint8_t*)(&recbuff), 1, 1000);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	return (0x7ff&recbuff);
}

void DRV8323_setupSpi()
{
//    volatile uint16_t temp;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); //PC5 is enable to motor controller
    HAL_Delay(1);
	DRV8323_writeSpi(ADR_DRV_CTRL, DRV8323regDrvCtrl);
	HAL_Delay(1);
	DRV8323_writeSpi(ADR_GATE_DRV_HS, DRV8323regGateDrvHS);
	HAL_Delay(2);
	DRV8323_writeSpi(ADR_GATE_DRV_LS, DRV8323regGateDrvLS);
	HAL_Delay(2);
	DRV8323_writeSpi(ADR_OCP_CTRL, DRV8323regOcpCtrl);
	HAL_Delay(2);
	DRV8323_writeSpi(ADR_CSA_CTRL, DRV8323regCsaCtrl);
	HAL_Delay(2);

//    temp = DRV8323_readSpi(ADR_FAULT_STAT);
//    temp = DRV8323_readSpi(ADR_VGS_STAT);
//    temp = DRV8323_readSpi(ADR_DRV_CTRL);
 //   temp = DRV8323_readSpi(ADR_GATE_DRV_HS);
//    temp = DRV8323_readSpi(ADR_GATE_DRV_LS);
 //   temp = DRV8323_readSpi(ADR_OCP_CTRL);
//    temp = DRV8323_readSpi(ADR_CSA_CTRL);
	return;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	if (x < in_min){
		x = in_min;
	}
	if (x > in_max){
		x = in_max;
	}
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}


/* PhaseA() B and C.
 * Sets the pin mode for the PWM output depending on the phase.
 */
void phaseA(int newPhase) {
	if (newPhase == pwm) {
		if(!slow_decay){
			LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
			GPIOA->BRR = GPIO_PIN_7;
		}else{
			LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE); // low
		}
		LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);  // high

	}

	if (newPhase == floating) {
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
		GPIOA->BRR = GPIO_PIN_7;
		LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOC->BRR = GPIO_PIN_0;
	}

	if (newPhase == lowside) {          // low mosfet on
		LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
		GPIOA->BSRR = GPIO_PIN_7;
		LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOC->BRR = GPIO_PIN_0;
	}

}

void phaseB(int newPhase) {

	if (newPhase == pwm) {
		if(!slow_decay){
			LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
			GPIOB->BRR = GPIO_PIN_0;
		}else{
			LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_ALTERNATE);
		}
		LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);

	}

	if (newPhase == floating) {            // floating
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOB->BRR = GPIO_PIN_0;
		LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
		GPIOC->BRR = GPIO_PIN_1;
	}

	if (newPhase == lowside) {              // lowside
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
		GPIOB->BSRR = GPIO_PIN_0;
		LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
		GPIOC->BRR = GPIO_PIN_1;
	}
}

void phaseC(int newPhase) {
	if (newPhase == pwm) {
		if(!slow_decay){
			LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
			GPIOB->BRR = GPIO_PIN_1;
		}else{
			LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_ALTERNATE);
		}
		LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_2, LL_GPIO_MODE_ALTERNATE);

	}

	if (newPhase == floating) {
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
		GPIOB->BRR = GPIO_PIN_1;
		LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
		GPIOC->BRR = GPIO_PIN_2;
	}

	if (newPhase == lowside) {
		LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
		GPIOB->BSRR = GPIO_PIN_1;
		LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
		GPIOC->BRR = GPIO_PIN_2;
	}

}


/*
 * sets phase A , B , C to either pwm , low or floating depending on the drive step
 */
void comStep(int newStep) {

	if (newStep == 1) {			//A-B
		phaseA(pwm);
		phaseB(lowside);
		phaseC(floating);
	}

	if (newStep == 2) {			// C-B
		phaseA(floating);
		phaseB(lowside);
		phaseC(pwm);
	}

	if (newStep == 3) {		// C-A
		phaseA(lowside);
		phaseB(floating);
		phaseC(pwm);
	}

	if (newStep == 4) {    // B-A
		phaseA(lowside);
		phaseB(pwm);
		phaseC(floating);
	}

	if (newStep == 5) {          // B-C
		phaseA(floating);
		phaseB(pwm);
		phaseC(lowside);
	}

	if (newStep == 6) {       // A-C
		phaseA(pwm);
		phaseB(floating);
		phaseC(lowside);
	}

}

void allOff() {          // all motor pwm output low
	phaseA(floating);
	phaseB(floating);
	phaseC(floating);
}

void fullBrake(){               // turns on lower mosfets to short motor windings and brake
	phaseA(lowside);
	phaseB(lowside);
	phaseC(lowside);
}


/*
 * Sets the comparator input to the correct one for the current step.
 */
void changeCompInput() {


	if (step == 1 || step == 4) {   // c floating
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH2;
	}

	if (step == 2 || step == 5) {     // a floating
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_IO1;
	}

	if (step == 3 || step == 6) {      // b floating
		hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_DAC1_CH1;
	}
	if (rising){
		hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_FALLING;   // polarity of comp output is reversed
	}else{                          // falling bemf
		hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING;
	}

	if (HAL_COMP_Init(&hcomp1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* commutate increments and changes the current commutation step and comparator input
 *  there are 6 steps for bldc drive per electrical revolution.
 */

void commutate() {

	if (forward == 1){
		step++;
		if (step > 6) {
			step = 1;
		}
		if (step == 1 || step == 3 || step == 5) {
			rising = 1;
		}
		if (step == 2 || step == 4 || step == 6) {
			rising = 0;
		}
	}
	if (forward == 0){
		step--;
		if (step < 1) {
			step = 6;
		}
		if (step == 1 || step == 3 || step == 5) {
			rising = 0;
		}
		if (step == 2 || step == 4 || step == 6) {
			rising = 1;
		}
	}
	comStep(step);
	changeCompInput();
}

/*

*/
void HAL_COMP_TriggerCallback(COMP_HandleTypeDef *hcomp) {
	if (TIM3->CNT < commutation_interval>>1){
		return;
	}
	/* Turn On LED3 */
		thiszctime = TIM3->CNT;
	//	GPIOA->BSRR = GPIO_PIN_15;
if (commutation_interval < 300 && bemf_counts < 300){         // startup failure
	HAL_COMP_Stop_IT(&hcomp1);
	input = 0;
			error = 2;
			return;
}
	if (compit > 20){
		HAL_COMP_Stop_IT(&hcomp1);
		error = 1;
		return;
	}
	compit +=1;
	while (TIM3->CNT - timestamp < filter_delay){

	}
	if (rising){
	//	advancedivisor = advancedivisorup;
		for (int i = 0; i < filter_level; i++){
		if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_HIGH){
	//		GPIOA->BRR = GPIO_PIN_15;
		return;
		}
		}
	}else{
	//	advancedivisor = advancedivisordown;
		for (int i = 0; i < filter_level; i++){
		if (HAL_COMP_GetOutputLevel(&hcomp1) == COMP_OUTPUTLEVEL_LOW){
	//		GPIOA->BRR = GPIO_PIN_15;

			return;
		}
		}

	}
	timestamp = TIM3->CNT;
	TIM3->CNT = 0 + timestamp - thiszctime;

	HAL_COMP_Stop_IT(&hcomp1);

	zctimeout = 0;

				commutation_interval = ((5 *commutation_interval) + thiszctime) / 6;    // TEST!   divide by two when tracking up down time independant
	//			bad_commutation = 0;
//			}
			advance = commutation_interval / advancedivisor;
			waitTime = commutation_interval /2   - advance ;
			blanktime = commutation_interval / 10;
		//	GPIOA->BRR = GPIO_PIN_15;

//		if(tempbrake){
//				HAL_COMP_Stop_IT(&hcomp1);
//				return;
//		}
		if (sensorless){
			while (TIM3->CNT  < waitTime){
			//	GPIOA->BSRR = GPIO_PIN_15;

			}
	//		TIM1->CNT = duty_cycle;
//			GPIOA->BRR = GPIO_PIN_15;
//			forcedcount = 0;
			compit = 0;
			commutate();
			while (TIM3->CNT  < waitTime + blanktime){
			}
		}

		lastzctime = thiszctime;
		if (bemf_counts < 200){
        bemf_counts++;
		}


//	}
//	GPIOA->BSRR = GPIO_PIN_15;
	if (HAL_COMP_Start_IT(&hcomp1) != HAL_OK) {
		/* Initialization Error */
		Error_Handler();
	}
//	GPIOA->BRR = GPIO_PIN_15;
}


void playStartupTune(){    // Sets the prescaler of PWM timer to 75 to put the frequency in the audible range
	TIM1->PSC = 75;
	TIM1->CCR1 = 5;        // keep the duty cycle low
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(1);            // put it on step one
	HAL_Delay(100);
	TIM1->PSC = 50;        // change pitch by changing prescaler
	HAL_Delay(100);
	TIM1->PSC = 25;
	HAL_Delay(100);
	allOff();
	TIM1->PSC = 1;         // put the prescaler back to normal
}

void playInputTune(){
	TIM1->PSC = 100;
	TIM1->CCR1 = 5;
	TIM1->CCR2 = 5;
	TIM1->CCR3 = 5;
	comStep(1);
	HAL_Delay(100);
	TIM1->PSC = 50;
	HAL_Delay(100);
	allOff();
	TIM1->PSC = 1;
}

void getADCs(){
	voltageraw = ADC1ConvertedValues[0];
	currentraw = ADC1ConvertedValues[1];

}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)
//{
//	getADCs();
//}

/* Checks the dma buffer for the smallest time increment to determine type of input */
void detectInput(){                         // legacy function, not needed unless using input capture to DMA buffer
	smallestnumber = 20000;
	servoPwm = 0;
	int lastnumber = dma_buffer[0];
	for ( int j = 1 ; j < input_buffer_size; j++){

		if((dma_buffer[j] - lastnumber) < smallestnumber){ // blank space
			smallestnumber = dma_buffer[j] - lastnumber;

		}
		lastnumber = dma_buffer[j];
	}
	if (smallestnumber > 1000 ){
		servoPwm = 1;
		TIM2->PSC = 71;
		TIM2->CNT = 65536;
	}

	if (smallestnumber == 0){
		inputSet = 0;
	}else{
		inputSet = 1;
		HAL_Delay(50);
	}
	HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, dma_buffer , 64);        // starts the input capture to DMA buffer of length 64
}


void computeServoInput(){                   // maps the servo pulse length to newinput



   if((dma_buffer[1] - dma_buffer[0] > 980 ) && ((dma_buffer[1] - dma_buffer[0]) < 2010)){ // servo zone
	   if((dma_buffer[2] - dma_buffer[1]) > 2500 ){          // blank space
		   servoraw = map((dma_buffer[1] - dma_buffer[0]), 1090, 2000, 0, 2000);
	   }
	} else if((dma_buffer[2] - dma_buffer[1] > 980 ) && ((dma_buffer[2] - dma_buffer[1]) < 2010)){ // servo zone
  	   if((dma_buffer[1] - dma_buffer[0]) > 2500 ){          // blank space
  		   servoraw = map((dma_buffer[2] - dma_buffer[1]), 1090, 2000, 0, 2000);
  	   }
  	} else{
  		servoraw = 1;       // not enough to arm but and below motor start threshold
  	}
   if (servoraw - newinput > max_servo_deviation){
   		newinput += max_servo_deviation;
   	}else if(newinput - servoraw > max_servo_deviation){
   		newinput -= max_servo_deviation;
   	}else{
   		newinput = servoraw;
   	}
}

void transferComplete(){      // called when DMA buffer is full, starts DMA transfer with reduced buffer size
	if (inputSet == 1){
		if  (servoPwm == 1){
			computeServoInput();
			signaltimeout = 0;
			TIM2->CNT = 0;
			HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_4, dma_buffer , 3);

		}
	}
}
void startMotor() {
	commutate_count++;
    char decaystate = slow_decay;
    sensorless = 0;
	if (running == 0){
		HAL_COMP_Stop_IT(&hcomp1);
		slow_decay = 1;


	commutate();
	HAL_Delay(2);

	commutation_interval = start_com_interval;
	TIM3->CNT = 0;
//	TIM2->CNT = 0;
//	TIM2->ARR = commutation_interval * 2;
	running = 1;
	if (HAL_COMP_Start_IT(&hcomp1) != HAL_OK) {
			/* Initialization Error */
			Error_Handler();
		}
	}

	slow_decay = decaystate;    // return to normal
	sensorless = 1;
	startupcountdown =0;
	bemf_counts = 0;

}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
        {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_COMP1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  MX_TIM16_Init();
  MX_SPI3_Init();

  /* USER CODE BEGIN 2 */
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
      HAL_TIM_Base_Start_IT(&htim16);
  	HAL_TIM_Base_Start(&htim3);


	        // plays a startup tune throught the motor

	if (HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_5) != HAL_OK)
	{
	//	_Error_Handler(__FILE__, __LINE__);
	}

	//  	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC1ConvertedValues, 2) != HAL_OK)
	//  		return 0;

	if(HAL_COMP_Start_IT(&hcomp1) != HAL_OK)            // start comparator in interrupt mode
	{
		/* Initialization Error */
		Error_Handler();
	}
	if(HAL_IWDG_Init(&hiwdg) != HAL_OK)          // start watch dog timer
	{
		/* Initialization Error */
		Error_Handler();
	}
  	DRV8323_setupSpi();
  	playStartupTune();
  	TIM1->CCR1 = 0;												// set duty cycle to low start amount
  	TIM1->CCR2 = 0;
  	TIM1->CCR3 = 0;
  	TIM1->CCR5 = pwm_settle;

if(bi_direction){
	newinput = 1005;
}

  	///test code
  	 // 	 LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
  	 // 	 LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
  	 //	LL_GPIO_SetPinMode(GPIOB, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);

  	/// HIGH SIDES////
  	 //	LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_0, LL_GPIO_MODE_OUTPUT);
  	 //	LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_1, LL_GPIO_MODE_OUTPUT);
  	 //	LL_GPIO_SetPinMode(GPIOC, GPIO_PIN_2, LL_GPIO_MODE_OUTPUT);
  	//  	phaseA(floating);
  	//  	phaseB(floating);
  	//  	phaseC(pwm);
  	//  	LL_GPIO_SetPinMode(GPIOA, GPIO_PIN_7, LL_GPIO_MODE_OUTPUT);
  	//  	GPIOA->BSRR = GPIO_PIN_7;



  	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(HAL_IWDG_Refresh(&hiwdg) != HAL_OK)                   // watchdog refresh
	{
		/* Refresh Error */
		Error_Handler();
	}

	  compit = 0;
	  count++;

	  if ( count > 50000) {
		  count = 0;
	  }

	/*
	 * The next section takes newinput and keeps it within rate of change limits,
	 * the variable newinput can come from any source.
	 */

		if (bi_direction) {
			//char oldbrake = brake;

			if (newinput > 1100) {
				if (forward == 0) {
					forward = 1;
					bemf_counts = 0;

				}
				adjusted_input = (newinput - 1050) * 3;

			}
			if (newinput < 780) {
				if (forward == 1) {
					forward = 0;
					bemf_counts = 0;

				}
				adjusted_input = ((780 - newinput) * 3) + 100;
			}

			if (newinput > 779 && newinput < 1101) {
				adjusted_input = 0;
			}
	}else{           // single direction only
		adjusted_input = newinput;
	}

//	  adjusted_input = newinput;
	  if (adjusted_input > 100){
		if (adjusted_input > 2000){
			adjusted_input = 2000;
		}
		if (adjusted_input >= input){
			if (adjusted_input - input > 1){
					input += 1;


			}else{
				input = adjusted_input;
			}
		}
		if (adjusted_input < input){
			if (input - adjusted_input > 1){
				input -=1 ;
			}else{
				input = adjusted_input;
			}
		}
	}else{
		input = adjusted_input;
	}

	if (inputSet == 0){
		HAL_Delay(10);                             //        set back to 10 after testing
		detectInput();

	}

	if (bemf_counts < 3 || commutation_interval > 3000) {
				filter_delay = 0;
				filter_level = 12;
			} else {
				filter_delay = 0;
				filter_level = 5;

			}
	advancedivisor = map((commutation_interval), 100, 8000, 4, 8);

	signaltimeout++;
	if (signaltimeout > signal_timeout_threshold) {
		signaltimeout = 0;
		input = 0;
		armed = 0;
		armedcount = 0;
		error = 1;
		inputSet = 0;
		TIM15->PSC=0;
		TIM16->PSC=0;
		servoPwm = 0;
		HAL_TIM_Base_Start_IT(&htim16);
	//	IC_buffer_size = 64;
		for (int i=0; i < 64; i++){
			dma_buffer[i] = 0;
		}
		HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, dma_buffer , 64);
		//	  duty_cycle = 0;          //mid point
	}


	if (!armed){                   // if not armed
		if ((inputSet == 1)&&(input == 0)){
			armedcount++;
			HAL_Delay(1);
			if (armedcount > 500){            // require a input to be stable at 0 for 500 ms
				commutation_interval = 0;
				armed = 1;
				playInputTune();
			}
		}
		if (input > 1){            // if the input goes over 1 reset armed count
			armedcount = 0;
		}
	}

	if (duty_cycle < 500 && commutation_interval < 200){         // stuck commutation at pwm speed
		zctimeout = zc_timeout_threshold;
	}

	if ((input > 100)&&(armed == 1)) {             // minimum input amount to start motor
		started = 1;
		duty_cycle = input - 15;                   // sets duty cycle, PWM
		if (duty_cycle > 1498){                      //  limit to 1400 for testing out of 1499
			duty_cycle = 1498;
		}
		if (bemf_counts < 30 ){
			if (duty_cycle > 500){
				duty_cycle = 500;
			}
			if (duty_cycle < 200){
				duty_cycle = 200;
			}
		}
		if (bemf_counts < 5 ){

					if (duty_cycle < start_power){
						duty_cycle = start_power;
					}
				}



		if (running){
			TIM1->CCR1 = duty_cycle;                // sets duty cycle for PWM output
			TIM1->CCR2 = duty_cycle;
			TIM1->CCR3 = duty_cycle;
			TIM1->CCR5 = pwm_settle;
		}
	}
	if (input <= 100) {                   //if input is less than 100 stop all, needs work for proper restart on the fly
		//HAL_COMP_Stop_IT(&hcomp1);
		started = 0;
	//	running = 0;
	//	commutation_interval = 0;
	//	duty_cycle = 120;
		if (!brake){
	//		allOff();
		}
		if(brake){
			fullBrake();
		}
		TIM1->CCR1 = 1;
		TIM1->CCR2 = 1;
		TIM1->CCR3 = 1;
	}


	if (TIM3->CNT > 40000){          // if the commutation timer goes past a normal commuatation length ZC's have timed out
		  running = 0;
		  started = 0;
		  TIM3->CNT = 0;

		}



	if (started == 1) {                           // Start the motor
		if (running == 0) {
			allOff();
			startMotor();  // safety on for input testing   ************************************************
		}
	}

//	if(startup_throttle_limit_countdown < startupcycles){           // to be tested, throttle limit for a short duration at startup
//		startup_throttle_limit_countdown++;
//		TIM1->CCR1 = start_power;
//		TIM1->CCR2 = start_power;
//		TIM1->CCR3 = start_power;
//
//	}

//	zc_timeout_threshold = 2000 - duty_cycle;               // sets the zero cross timeout threshold lower at higher duty cycles
}

  /* USER CODE END 3 */

}

/*
** This notice applies to any and all portions of this file
* that are not between comment pairs USER CODE BEGIN and
* USER CODE END. Other portions of this file, whether
* inserted by the user or by software development tools
* are owned by their respective copyright owners.
*
* COPYRIGHT(c) 2019 STMicroelectronics
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM16
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* COMP1 init function */
static void MX_COMP1_Init(void)
{

  hcomp1.Instance = COMP1;
  hcomp1.Init.InvertingInput = COMP_INVERTINGINPUT_IO1;
  hcomp1.Init.NonInvertingInput = COMP_NONINVERTINGINPUT_IO1;
  hcomp1.Init.Output = COMP_OUTPUT_NONE;
  hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp1.Init.BlankingSrce = COMP_BLANKINGSRCE_TIM1OC5;
  hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_IT_RISING_FALLING;
  if (HAL_COMP_Init(&hcomp1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
	hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
	hiwdg.Init.Reload = 2000;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1499;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
    sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 120;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65536;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 20;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 10;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 5000;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  LL_GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);

  /**/
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_5);

  /**/
  LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);

  /**/
  LL_GPIO_ResetOutputPin(GPIOD, LL_GPIO_PIN_2);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  GPIO_InitStruct.Pin = LL_GPIO_PIN_2;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
