/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS_CLI.h"
#include "arm_const_structs.h"
#include "cmsis_os.h"
#include "main.h"
#include "message_buffer.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

UART_HandleTypeDef hlpuart1;
DMA_HandleTypeDef hdma_lpuart1_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

SemaphoreHandle_t mutex_uart;
MessageBufferHandle_t message_buffer;
SemaphoreHandle_t sem_tx_uart;
SemaphoreHandle_t sem_adc;
QueueHandle_t uart_rx_q;

uint16_t sin_wave[256] = {
    2048, 2098, 2148, 2199, 2249, 2299, 2349, 2399, 2448, 2498, 2547, 2596,
    2644, 2692, 2740, 2787, 2834, 2880, 2926, 2971, 3016, 3060, 3104, 3147,
    3189, 3230, 3271, 3311, 3351, 3389, 3427, 3464, 3500, 3535, 3569, 3602,
    3635, 3666, 3697, 3726, 3754, 3782, 3808, 3833, 3857, 3880, 3902, 3923,
    3943, 3961, 3979, 3995, 4010, 4024, 4036, 4048, 4058, 4067, 4074, 4081,
    4086, 4090, 4093, 4095, 4095, 4094, 4092, 4088, 4084, 4078, 4071, 4062,
    4053, 4042, 4030, 4017, 4002, 3987, 3970, 3952, 3933, 3913, 3891, 3869,
    3845, 3821, 3795, 3768, 3740, 3711, 3681, 3651, 3619, 3586, 3552, 3517,
    3482, 3445, 3408, 3370, 3331, 3291, 3251, 3210, 3168, 3125, 3082, 3038,
    2994, 2949, 2903, 2857, 2811, 2764, 2716, 2668, 2620, 2571, 2522, 2473,
    2424, 2374, 2324, 2274, 2224, 2174, 2123, 2073, 2022, 1972, 1921, 1871,
    1821, 1771, 1721, 1671, 1622, 1573, 1524, 1475, 1427, 1379, 1331, 1284,
    1238, 1192, 1146, 1101, 1057, 1013, 970,  927,  885,  844,  804,  764,
    725,  687,  650,  613,  578,  543,  509,  476,  444,  414,  384,  355,
    327,  300,  274,  250,  226,  204,  182,  162,  143,  125,  108,  93,
    78,   65,   53,   42,   33,   24,   17,   11,   7,    3,    1,    0,
    0,    2,    5,    9,    14,   21,   28,   37,   47,   59,   71,   85,
    100,  116,  134,  152,  172,  193,  215,  238,  262,  287,  313,  341,
    369,  398,  429,  460,  493,  526,  560,  595,  631,  668,  706,  744,
    784,  824,  865,  906,  948,  991,  1035, 1079, 1124, 1169, 1215, 1261,
    1308, 1355, 1403, 1451, 1499, 1548, 1597, 1647, 1696, 1746, 1796, 1846,
    1896, 1947, 1997, 2047};

uint16_t sin_wave_3rd_harmonic[256] = {
    2048, 2136, 2224, 2311, 2398, 2484, 2569, 2652, 2734, 2814, 2892, 2968,
    3041, 3112, 3180, 3245, 3308, 3367, 3423, 3476, 3526, 3572, 3615, 3654,
    3690, 3723, 3752, 3778, 3800, 3819, 3835, 3848, 3858, 3866, 3870, 3872,
    3871, 3869, 3864, 3857, 3848, 3838, 3827, 3814, 3801, 3786, 3771, 3756,
    3740, 3725, 3709, 3694, 3679, 3665, 3652, 3639, 3628, 3617, 3608, 3600,
    3594, 3589, 3585, 3584, 3583, 3584, 3587, 3591, 3597, 3604, 3613, 3622,
    3633, 3645, 3658, 3672, 3686, 3701, 3717, 3732, 3748, 3764, 3779, 3794,
    3808, 3821, 3833, 3844, 3853, 3860, 3866, 3870, 3872, 3871, 3868, 3862,
    3854, 3842, 3828, 3810, 3789, 3765, 3738, 3707, 3673, 3635, 3594, 3549,
    3501, 3450, 3396, 3338, 3277, 3213, 3146, 3077, 3005, 2930, 2853, 2774,
    2693, 2611, 2527, 2441, 2355, 2268, 2180, 2092, 2003, 1915, 1827, 1740,
    1654, 1568, 1484, 1402, 1321, 1242, 1165, 1090, 1018, 949,  882,  818,
    757,  699,  645,  594,  546,  501,  460,  422,  388,  357,  330,  306,
    285,  267,  253,  241,  233,  227,  224,  223,  225,  229,  235,  242,
    251,  262,  274,  287,  301,  316,  331,  347,  363,  378,  394,  409,
    423,  437,  450,  462,  473,  482,  491,  498,  504,  508,  511,  512,
    511,  510,  506,  501,  495,  487,  478,  467,  456,  443,  430,  416,
    401,  386,  370,  355,  339,  324,  309,  294,  281,  268,  257,  247,
    238,  231,  226,  224,  223,  225,  229,  237,  247,  260,  276,  295,
    317,  343,  372,  405,  441,  480,  523,  569,  619,  672,  728,  787,
    850,  915,  983,  1054, 1127, 1203, 1281, 1361, 1443, 1526, 1611, 1697,
    1784, 1871, 1959, 2047};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_LPUART1_UART_Init(void);
void StartDefaultTask(void const *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char uart_data;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
    char data = huart->Instance->RDR;
    xQueueSendFromISR(uart_rx_q, &data, &pxHigherPriorityTaskWoken);
    HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)&uart_data, 1);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sem_tx_uart, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

// Antiga UART_RX_RTOS - Retorna os dados recebidos pela UART
BaseType_t get_char_from_uart(char *pData, TickType_t timeout) {
    return xQueueReceive(uart_rx_q, pData, timeout);
}

void print_string(char *string, TickType_t timeout) {
    if (xSemaphoreTake(mutex_uart, timeout) == pdTRUE) {
        (void)xMessageBufferSend(message_buffer, string, strlen(string),
                                 portMAX_DELAY);
        xSemaphoreGive(mutex_uart);
    }
}

void print_char(char string, TickType_t timeout) {
    if (xSemaphoreTake(mutex_uart, timeout) == pdTRUE) {
        (void)xMessageBufferSend(message_buffer, &string, 1, portMAX_DELAY);
        xSemaphoreGive(mutex_uart);
    }
}

#define BUFFER_SIZE 512

static void print_task(void *params) {
    char buffer[BUFFER_SIZE];

    while (1) {
        size_t size = xMessageBufferReceive(message_buffer, buffer, BUFFER_SIZE,
                                            portMAX_DELAY);
        if (size) {
            HAL_UART_Transmit_DMA(&hlpuart1, (uint8_t *)buffer, size);
            xSemaphoreTake(sem_tx_uart, portMAX_DELAY);
        }
    }
}

void adc_task(void *param) {
    uint16_t adcBuffer[256];
    float ReIm[256 * 2];
    float mod[256];
    // uint32_t count = 0;

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 256);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                      (const uint32_t *)sin_wave_3rd_harmonic, 256,
                      DAC_ALIGN_12B_R);

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim3);

    while (1) {
        xSemaphoreTake(sem_adc, portMAX_DELAY);

        // volatile TickType_t start = xTaskGetTickCount();

        int k = 0;
        for (int i = 0; i < 256; i++) {
            ReIm[k] = (float)adcBuffer[i] * 0.0008056640625f;
            ReIm[k + 1] = 0.0;
            k += 2;
        }

        arm_cfft_f32(&arm_cfft_sR_f32_len256, ReIm, 0, 1);
        arm_cmplx_mag_f32(ReIm, mod, 256);
        arm_scale_f32(mod, 0.0078125, mod, 128);
        mod[0] = mod[0] * 0.5;

        volatile float fund_phase = atan2f(ReIm[3], ReIm[2]) * 180 /
                                    M_PI;  // Fase R da harmonica fundamental
        (void)fund_phase;

        // count++;
        // if (count >= 30){
        //	count = 0;
        // xQueueSend(fft_q, &mod, 10);
        //}
        // volatile TickType_t stop = xTaskGetTickCount();
        //(void)fund_phase;
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    portBASE_TYPE pxHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sem_adc, &pxHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

volatile int dac_counter = 0;
volatile int flag = 0;
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hadc) {
#if 1
    dac_counter++;
    if (dac_counter >= 120) {
        dac_counter = 0;
        HAL_TIM_Base_Stop(&htim2);
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);

        if (flag == 0) {
            HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                              (uint32_t *)sin_wave_3rd_harmonic, 256,
                              DAC_ALIGN_12B_R);
            flag = 1;
        } else {
            HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)sin_wave, 256,
                              DAC_ALIGN_12B_R);
            flag = 0;
        }
        HAL_TIM_Base_Start(&htim2);
    }
#endif
}

/* ---------------- Get Installed Tasks ---------------- */

static BaseType_t getInstalledTasksFunction(char *pcWriteBuffer,
                                            size_t xWriteBufferLen,
                                            const char *pcCommandString) {
    (void)xWriteBufferLen;
    vTaskList(pcWriteBuffer);
    return pdFALSE;

    // static BaseType_t state = 0;

    // if (state == 0) {
    //     // char *head = "Name		State  Priority  Stack
    //     Number\n\r"; (void)xWriteBufferLen; vTaskList(pcWriteBuffer);
    //     // strcpy(pcWriteBuffer, head);
    //     // vTaskList(&pcWriteBuffer[strlen(head)]);
    //     state = 1;
    //     return pdTRUE;
    // } else {
    //     state = 0;
    //     strcpy(pcWriteBuffer, "\n\r");
    //     return pdFALSE;
    // }
}

static const CLI_Command_Definition_t xGetInstalledTasksCommand = {
    "tasks", "tasks: list all tasks\n\n\r", getInstalledTasksFunction, 0};

/* ---------------- Get Runtime Info ---------------- */

static BaseType_t getRuntimeStatsFunction(char *pcWriteBuffer,
                                          size_t xWriteBufferLen,
                                          const char *pcCommandString) {
    char *head = "Name		Abs Time      % Time\n\r";
    (void)xWriteBufferLen;
    strcpy(pcWriteBuffer, head);
    vTaskGetRunTimeStats(&pcWriteBuffer[strlen(head)]);
    return pdFALSE;
}

static const CLI_Command_Definition_t xGetRuntimeCommand = {
    "runtime", "runtime: Show the runtime info\n\n\r", getRuntimeStatsFunction,
    0};

/* ---------------- Change Sin Wave ---------------- */
uint16_t sin_wave[256];
uint16_t sin_wave_3rd_harmonic[256];
static BaseType_t changeWaveFunction(char *pcWriteBuffer,
                                     size_t xWriteBufferLen,
                                     const char *pcCommandString) {
    BaseType_t parameter_lenght;
    const char *parameter =
        FreeRTOS_CLIGetParameter(pcCommandString, 1, &parameter_lenght);

    if (!strcmp(parameter, "sine")) {
        HAL_TIM_Base_Stop(&htim2);
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t *)sin_wave, 256,
                          DAC_ALIGN_12B_R);
        HAL_TIM_Base_Start(&htim2);
        strcpy(pcWriteBuffer, "Sine Signal set\n\r");
    } else if (!strcmp(parameter, "sine3rd")) {
        HAL_TIM_Base_Stop(&htim2);
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1,
                          (uint32_t *)sin_wave_3rd_harmonic, 256,
                          DAC_ALIGN_12B_R);
        HAL_TIM_Base_Start(&htim2);
        strcpy(pcWriteBuffer, "Sine3rd Signal set\n\r");
    } else {
        strcpy(pcWriteBuffer, "Invalid wave signal!\n\r");
    }
    return pdFALSE;
}

static const CLI_Command_Definition_t xChangeWaveCommand = {
    "wave", "wave: Changes the DAC wave signal (params: sine | sine3rd)\n\n\r",
    changeWaveFunction, 1};

/* ---------------- Clear Terminal ---------------- */

static BaseType_t clearTerminalFunction(char *pcWriteBuffer,
                                        size_t xWriteBufferLen,
                                        const char *pcCommandString) {
    strcpy(pcWriteBuffer, "\033[H\033[J\n\r");
    return pdFALSE;
}

static const CLI_Command_Definition_t xClearTerminalCommand = {
    "clear", "clear: Clear the terminal\n\n\r", clearTerminalFunction, 0};

/* ---------------- Terminal Task ---------------- */

#define MAX_INPUT_LENGTH 50
#define MAX_OUTPUT_LENGTH 100

void terminal_task(void *params) {
    int8_t cRxedChar, cInputIndex = 0;
    BaseType_t xMoreDataToFollow;

    /* Buffers de entrada e saída */
    static int8_t pcInputString[MAX_INPUT_LENGTH];
    static int8_t pcOutputString[MAX_OUTPUT_LENGTH];

    FreeRTOS_CLIRegisterCommand(&xGetInstalledTasksCommand);
    FreeRTOS_CLIRegisterCommand(&xGetRuntimeCommand);
    FreeRTOS_CLIRegisterCommand(&xChangeWaveCommand);
    FreeRTOS_CLIRegisterCommand(&xClearTerminalCommand);

    print_string("----- FreeRTOS Terminal -----\r\n\n", portMAX_DELAY);

    /* Recepção de 1byte pela uart */
    HAL_UART_Receive_IT(&hlpuart1, (uint8_t *)&uart_data, 1);
    hlpuart1.RxISR = HAL_UART_RxCpltCallback;

    while (1) {
        // Espera indefinidamente por um caractere
        get_char_from_uart(&cRxedChar, portMAX_DELAY);

        if (cRxedChar == '\r') {
            /* Tecla "Enter" seja pressionada */
            print_string("\r\n", portMAX_DELAY);

            /* Execução do comando inserido ao pressionar enter: */
            do {
                xMoreDataToFollow = FreeRTOS_CLIProcessCommand(
                    pcInputString,    /* string do comando.*/
                    pcOutputString,   /* buffer de saída. */
                    MAX_OUTPUT_LENGTH /* Tamanho do buffer de saída. */
                );

                print_string(pcOutputString, portMAX_DELAY);
            } while (xMoreDataToFollow != pdFALSE);

            /* Limpa a string de entrada */
            cInputIndex = 0;
            memset(pcInputString, 0x00, MAX_INPUT_LENGTH);

        } else {
            if (cRxedChar == '\n') {
                // Ignora o \n
            } else if (cRxedChar == '\b') {
                /* Tratamento do backspace */
                if (cInputIndex > 0) {
                    cInputIndex--;
                    pcInputString[cInputIndex] = '\0';
                    print_char(cRxedChar, portMAX_DELAY);
                }
            } else {
                // Adiciona o caractere na string de entrada
                if (cInputIndex < MAX_INPUT_LENGTH) {
                    pcInputString[cInputIndex] = cRxedChar;
                    cInputIndex++;
                }
                print_char(cRxedChar, portMAX_DELAY);
            }
        }
    }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU
     * Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the
     * Systick. */
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
    MX_ADC1_Init();
    MX_TIM3_Init();
    MX_DAC1_Init();
    MX_TIM2_Init();
    MX_LPUART1_UART_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* USER CODE BEGIN RTOS_MUTEX */
    mutex_uart = xSemaphoreCreateMutex();

    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */

    sem_tx_uart = xSemaphoreCreateBinary();
    sem_adc = xSemaphoreCreateBinary();

    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */

    message_buffer = xMessageBufferCreate(BUFFER_SIZE);
    uart_rx_q = xQueueCreate(32, sizeof(char));
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */

    (void)xTaskCreate(terminal_task, "Console", 256, NULL, 3, NULL);
    (void)xTaskCreate(print_task, "Print Task", 256, NULL, 10, NULL);
    (void)xTaskCreate(adc_task, "ADC", 2048, NULL, 6, NULL);

    /* USER CODE END RTOS_THREADS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {
    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.GainCompensation = 0;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void) {
    /* USER CODE BEGIN DAC1_Init 0 */

    /* USER CODE END DAC1_Init 0 */

    DAC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN DAC1_Init 1 */

    /* USER CODE END DAC1_Init 1 */

    /** DAC Initialization
     */
    hdac1.Instance = DAC1;
    if (HAL_DAC_Init(&hdac1) != HAL_OK) {
        Error_Handler();
    }

    /** DAC channel OUT1 config
     */
    sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
    sConfig.DAC_DMADoubleDataMode = DISABLE;
    sConfig.DAC_SignedFormat = DISABLE;
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
    sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_BOTH;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN DAC1_Init 2 */

    /* USER CODE END DAC1_Init 2 */
}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void) {
    /* USER CODE BEGIN LPUART1_Init 0 */

    /* USER CODE END LPUART1_Init 0 */

    /* USER CODE BEGIN LPUART1_Init 1 */

    /* USER CODE END LPUART1_Init 1 */
    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = 115200;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) !=
        HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN LPUART1_Init 2 */

    /* USER CODE END LPUART1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {
    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 11067;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) !=
        HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {
    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 11067;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) !=
        HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMAMUX1_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    /* DMA1_Channel2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    /* DMA2_Channel2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument) {
    /* USER CODE BEGIN 5 */
    /* Infinite loop */
    for (;;) {
        osDelay(1);
    }
    /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */

    /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state
     */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line
       number, ex: printf("Wrong parameters value: file %s on line %d\r\n",
       file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
