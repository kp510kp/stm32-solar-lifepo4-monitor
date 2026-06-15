/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Production-Grade Solar Battery Monitor Core System
  * @note           : Corrected Single-Pass Inverse Calibration & Safe Deadband
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include "ff.h"
#include "ili9488.h"

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
FATFS FatFs;
FIL fil;
FRESULT fres;
char buffer[150];    /* Safe transmission buffer to prevent array overflows */

/* --- HARDWARE CONFIGURATION TARGET SELECTION --- */
#define HARDWARE_MODE         0   /* 1 = DFRobot ADS1115 (Direct), 0 = Niraj's Op-Amp Stacked Board */

#if (HARDWARE_MODE == 1)
    #define OPAMP_GAIN_FACTOR    1.0f             /* Unity gain for isolated module */
#else
    #define OPAMP_GAIN_FACTOR    0.7333333333f    /* Attenuation factor for Niraj Op-Amp board */
#endif

#define SAMPLES_PER_SECOND   12                   /* Safe rate to prevent SD/TFT lag (Tested with Professor) */
#define ADS1115_ADDR        (0x48 << 1)
#define REG_ADS_CONV        0x00
#define REG_ADS_CONFIG      0x01

#define INA226_ADDR         (0x40 << 1)
#define REG_CONFIG          0x00
#define REG_BUS_VOLT        0x02

/* --- APPROVED INVERSE CALIBRATION CONSTANTS --- */
const float M_VOLTAGE = 0.999902327f;
const float C_VOLTAGE = 0.000010103f;
const float M_CURRENT = 1.026593f;
const float C_CURRENT = -0.003363f;  /* Negative offset */

/* Global measurement registers */
float v1 = 0.0f, v2 = 0.0f, v3 = 0.0f, v4 = 0.0f;
float current_A = 0.0f;
float bus_voltage_V = 0.0f;

char time_string[30];
char current_filename[13] = "LOG000.CSV";
uint8_t sd_ok = 0;
uint32_t logging_start_time = 0;
char i2c_display_line[100] = "I2C Nodes:";

/* Color definitions for ILI9488 UI rendering */
#define UI_DIVIDER_COLOR     0x7BEF
#define ILI9488_BLUE         0x001F
#define ILI9488_BLACK        0x0000
#define ILI9488_CYAN         0x07FF
#define ILI9488_YELLOW       0xFFE0
#define ILI9488_GREEN        0x07E0
#define ILI9488_WHITE        0xFFFF
#define ILI9488_RED          0xF800
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

/* USER CODE BEGIN PFP */
void INA_Setup(void);
void INA_ReadData(void);
float Read_ADS1115_Channel(uint8_t channel);
void CreateNewLogFile(void);
void I2C_Scan_All_Devices(void);
void I2C_Scan_DisplayOnly(void);
void Custom_DrawFrameBorders(uint16_t color);
void Set_SPI_Speed(uint32_t prescaler);
/* USER CODE END PFP */

void Set_SPI_Speed(uint32_t prescaler)
{
    __HAL_SPI_DISABLE(&hspi1);
    hspi1.Init.BaudRatePrescaler = prescaler;
    HAL_SPI_Init(&hspi1);
    __HAL_SPI_ENABLE(&hspi1);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  INA_Setup();

  /* Establish initial state for SPI Chip Select lines */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   /* Deselect TFT */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);  /* Deselect SD Card */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);   /* Backlight ON */
  HAL_Delay(200);

  /* Transmit hardware profile via UART */
  #if (HARDWARE_MODE == 1)
      sprintf(buffer, "System Init: DFRobot ADS1115 Mode (Direct) Active\r\n");
  #else
      sprintf(buffer, "System Init: Op-Amp Mixer Mode (Calculated) Active\r\n");
  #endif
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);

  I2C_Scan_All_Devices();

  /* --- SD CARD INITIALIZATION --- */
  Set_SPI_Speed(SPI_BAUDRATEPRESCALER_64);
  f_mount(NULL, "", 0);
  HAL_Delay(100);

  uint8_t dummy = 0xFF;
  uint8_t response = 0xFF;
  uint8_t cmd0[6] = {0x40, 0, 0, 0, 0, 0x95};

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  for (int i = 0; i < 20; i++) {
      HAL_SPI_Transmit(&hspi1, &dummy, 1, 50);
  }

  HAL_SPI_Transmit(&hspi1, cmd0, 6, 50);
  for (int i = 0; i < 200; i++) {
      HAL_SPI_TransmitReceive(&hspi1, &dummy, &response, 1, 50);
      if (response != 0xFF) break;
  }
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(20);

  fres = f_mount(&FatFs, "", 1);
  if (fres == FR_OK)
  {
      sd_ok = 1;
      logging_start_time = HAL_GetTick();
      CreateNewLogFile();
  }
  else
  {
      sd_ok = 0;
      sprintf(buffer, "SD Mount Failed! Error: %d\r\n", fres);
      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
  }

  /* --- TFT DISPLAY INITIALIZATION --- */
  Set_SPI_Speed(SPI_BAUDRATEPRESCALER_2);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(100);

  ILI9488_Init();
  ILI9488_FillScreen(ILI9488_BLACK);

  Custom_DrawFrameBorders(ILI9488_BLUE);
  ILI9488_FillRect(5, 45,  310, 2, UI_DIVIDER_COLOR);
  ILI9488_FillRect(5, 135, 310, 2, UI_DIVIDER_COLOR);
  ILI9488_FillRect(5, 295, 310, 2, UI_DIVIDER_COLOR);
  ILI9488_FillRect(5, 420, 310, 2, UI_DIVIDER_COLOR);

  ILI9488_Print(20, 15, "SOLAR BATTERY MONITOR", ILI9488_CYAN, ILI9488_BLACK, 2);
  ILI9488_Print(15, 60, "CURRENT:", ILI9488_YELLOW, ILI9488_BLACK, 2);
  ILI9488_Print(15, 100, "BUS VOLT:", ILI9488_CYAN, ILI9488_BLACK, 2);
  ILI9488_Print(15, 150, "BATTERY_1:", ILI9488_GREEN, ILI9488_BLACK, 2);
  ILI9488_Print(15, 185, "BATTERY_2:", ILI9488_GREEN, ILI9488_BLACK, 2);
  ILI9488_Print(15, 220, "BATTERY_3:", ILI9488_GREEN, ILI9488_BLACK, 2);
  ILI9488_Print(15, 255, "BATTERY_4:", ILI9488_GREEN, ILI9488_BLACK, 2);
  ILI9488_Print(15, 315, "SD STATUS:", ILI9488_WHITE, ILI9488_BLACK, 2);
  ILI9488_Print(15, 350, "LOG FILE :", ILI9488_WHITE, ILI9488_BLACK, 2);
  ILI9488_Print(15, 385, "RUN TIME :", ILI9488_WHITE, ILI9488_BLACK, 2);

  /* CRITICAL CORRECTION: Safe Startup State - Keep load OFF (HIGH) initially to protect circuits */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);

  /* Time-slice scheduler variables */
  uint32_t last_sample_time = HAL_GetTick();
  uint32_t sample_count = 0;
  uint32_t sample_interval_ms = 1000 / SAMPLES_PER_SECOND;

  /* Accumulators for filtering */
  float screen_sum_v1 = 0.0f, screen_sum_v2 = 0.0f, screen_sum_v3 = 0.0f, screen_sum_v4 = 0.0f;
  float screen_sum_current = 0.0f, screen_sum_bus_volt = 0.0f;

  float final_avg_v1 = 0.0f, final_avg_v2 = 0.0f, final_avg_v3 = 0.0f, final_avg_v4 = 0.0f;
  float final_avg_curr = 0.0f, final_avg_bus = 0.0f;

  uint8_t display_task_state = 0;
  uint8_t screen_update_pending = 0;
  char log_write_buffer[10240] = {0};

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint32_t current_tick = HAL_GetTick();

      /* --- STEP 1: DETERMINISTIC TELEMETRY SAMPLING SLOT --- */
      if (current_tick - last_sample_time >= sample_interval_ms)
      {
          last_sample_time += sample_interval_ms;

          /* Isolate SPI lines to eliminate bus contention during sensor read */
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);

          /* Read current & bus voltage (Internally calibrated inside function now) */
          INA_ReadData();

          /* Read raw uncalibrated values from ADS1115 Channels */
          float raw_ch0 = Read_ADS1115_Channel(0);
          float raw_ch1 = Read_ADS1115_Channel(1);
          float raw_ch2 = Read_ADS1115_Channel(2);
          float raw_ch3 = Read_ADS1115_Channel(3);

          /* Noise floor hardware clamp */
          if (raw_ch0 < 0.001f) raw_ch0 = 0.0f;
          if (raw_ch1 < 0.001f) raw_ch1 = 0.0f;
          if (raw_ch2 < 0.001f) raw_ch2 = 0.0f;
          if (raw_ch3 < 0.001f) raw_ch3 = 0.0f;

          /* --- SINGLE-PASS INVERSE CALIBRATION MAPPING --- */
          #if (HARDWARE_MODE == 1)
              /* Direct Mode Mapping */
              v1 = raw_ch0;
              v2 = raw_ch1;
              v3 = raw_ch2;
              v4 = raw_ch3;
          #else
              /* Single battery connected to Op-Amp Channel A3 with Inverse Calibration Matrix */
              v4 = (raw_ch3 - C_VOLTAGE) / M_VOLTAGE;
              v1 = 0.0f;
              v2 = 0.0f;
              v3 = 0.0f;
          #endif

          /* Process running accumulators */
          screen_sum_v1 += v1;
          screen_sum_v2 += v2;
          screen_sum_v3 += v3;
          screen_sum_v4 += v4;
          screen_sum_current += current_A;
          screen_sum_bus_volt += bus_voltage_V;
          sample_count++;

          /* Generate ISO Timestamp string */
          uint32_t total_sec = (current_tick - logging_start_time) / 1000;
          uint32_t days    = total_sec / 86400;
          uint32_t hours   = (total_sec % 86400) / 3600;
          uint32_t minutes = (total_sec % 3600) / 60;
          uint32_t seconds = total_sec % 60;
          uint32_t msec    = (current_tick - logging_start_time) % 1000;

          sprintf(time_string, "%03lu %02lu:%02lu:%02lu.%03lu", days, hours, minutes, seconds, msec);

          /* 1. Format calibrated data for SD Card block storage buffer */
          sprintf(buffer, "%s,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n",
                  time_string, v1, v2, v3, v4, current_A, bus_voltage_V);
          strcat(log_write_buffer, buffer);

          /* 2. Enhanced safe formatting using snprintf for Serial Terminal output */
          char serial_buffer[200];
          snprintf(serial_buffer, sizeof(serial_buffer),
                   "T:%s | v1:%.5f, v2:%.5f, v3:%.5f, v4:%.5f | Bus_V:%.5f | I:%.5f\r\n",
                   time_string, v1, v2, v3, v4, bus_voltage_V, current_A);
          HAL_UART_Transmit(&huart2, (uint8_t*)serial_buffer, strlen(serial_buffer), 20);

          /* Safety Interlock: Cut off load if Cell 4 drops below 2.5V (Adjusted for Active Channel) */
          if (v4 <= 2.5000f && v4 > 0.2f)
          {
              /* Set pins HIGH to turn off Active-Low Relays instantly */
              HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_SET);
          }

          /* Execute buffered block-writes to SD card */
          if (sample_count >= SAMPLES_PER_SECOND)
          {
              float divisor = (float)sample_count;
              final_avg_v1 = screen_sum_v1 / divisor;
              final_avg_v2 = screen_sum_v2 / divisor;
              final_avg_v3 = screen_sum_v3 / divisor;
              final_avg_v4 = screen_sum_v4 / divisor;
              final_avg_curr = screen_sum_current / divisor;
              final_avg_bus  = screen_sum_bus_volt / divisor;

              /* Flush accumulator buffers */
              screen_sum_v1 = 0.0f; screen_sum_v2 = 0.0f; screen_sum_v3 = 0.0f; screen_sum_v4 = 0.0f;
              screen_sum_current = 0.0f; screen_sum_bus_volt = 0.0f;
              sample_count = 0;

              /* --- ATOMIC STORAGE TRANSACTION PHASE --- */
              if (sd_ok)
              {
                  Set_SPI_Speed(SPI_BAUDRATEPRESCALER_16);
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);

                  fres = f_open(&fil, current_filename, FA_OPEN_ALWAYS | FA_WRITE);
                  if (fres == FR_OK) {
                      UINT bytesWritten;
                      f_lseek(&fil, f_size(&fil));
                      f_write(&fil, log_write_buffer, strlen(log_write_buffer), &bytesWritten);
                      f_close(&fil);
                  } else {
                      sd_ok = 0;
                  }
                  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
                  Set_SPI_Speed(SPI_BAUDRATEPRESCALER_2);
              }

              log_write_buffer[0] = '\0';
              screen_update_pending = 1;
          }
      }

      /* --- STEP 2: SAFE DISPLAY UPDATE SLOT --- */
      if (screen_update_pending && (sample_interval_ms - (HAL_GetTick() - last_sample_time) > 40))
      {
          screen_update_pending = 0;

          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

          /* Refresh Metrics */
          uint32_t total_sec = (HAL_GetTick() - logging_start_time) / 1000;
          uint32_t days    = total_sec / 86400;
          uint32_t hours   = (total_sec % 86400) / 3600;
          uint32_t minutes = (total_sec % 3600) / 60;
          uint32_t seconds = total_sec % 60;
          sprintf(buffer, "%03lu %02lu:%02lu:%02lu", days, hours, minutes, seconds);
          ILI9488_Print(160, 385, buffer, ILI9488_YELLOW, ILI9488_BLACK, 2);

          sprintf(buffer, "%.5f A   ", final_avg_curr);
          ILI9488_Print(160, 60, buffer, ILI9488_WHITE, ILI9488_BLACK, 2);
          sprintf(buffer, "%.5f V   ", final_avg_bus);
          ILI9488_Print(160, 100, buffer, ILI9488_WHITE, ILI9488_BLACK, 2);

          sprintf(buffer, "%.5f V    ", final_avg_v1);
          ILI9488_Print(160, 150, buffer, ILI9488_WHITE, ILI9488_BLACK, 2);
          sprintf(buffer, "%.5f V    ", final_avg_v2);
          ILI9488_Print(160, 185, buffer, ILI9488_WHITE, ILI9488_BLACK, 2);
          sprintf(buffer, "%.5f V    ", final_avg_v3);
          ILI9488_Print(160, 220, buffer, ILI9488_WHITE, ILI9488_BLACK, 2);
          sprintf(buffer, "%.5f V    ", final_avg_v4);
          ILI9488_Print(160, 255, buffer, ILI9488_WHITE, ILI9488_BLACK, 2);

          switch (display_task_state)
          {
              case 0:
                  if (sd_ok) {
                      ILI9488_Print(160, 315, "ACTIVE  ", ILI9488_GREEN, ILI9488_BLACK, 2);
                      sprintf(buffer, "%s     ", current_filename);
                      ILI9488_Print(160, 350, buffer, ILI9488_CYAN, ILI9488_BLACK, 2);
                  } else {
                      ILI9488_Print(160, 315, "ERROR   ", ILI9488_RED, ILI9488_BLACK, 2);
                      ILI9488_Print(160, 350, "NONE        ", ILI9488_CYAN, ILI9488_BLACK, 2);
                  }
                  display_task_state = 1;
                  break;

              case 1:
                  I2C_Scan_DisplayOnly();
                  sprintf(buffer, "%-35s", i2c_display_line);
                  ILI9488_Print(15, 440, buffer, ILI9488_WHITE, ILI9488_BLACK, 2);
                  display_task_state = 0;
                  break;
          }
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
      }
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Reads conversion registers from ADS1115 ADC converter.
  * @param  channel: Targeting AIN pin configuration (0 to 3).
  * @retval Scaled float uncalibrated measured voltage representation.
  */
float Read_ADS1115_Channel(uint8_t channel)
{
    uint8_t config[3] = {REG_ADS_CONFIG, 0x00, 0xE3};
    uint8_t rx_data[2] = {0};

    switch(channel)
    {
        case 0: config[1] = 0xC2; break;
        case 1: config[1] = 0xD2; break;
        case 2: config[1] = 0xE2; break;
        case 3: config[1] = 0xF2; break;
        default: return 0.0f;
    }

    if (HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, config, 3, 10) != HAL_OK) return -1.0f;
    HAL_Delay(3);

    uint8_t reg_pointer = REG_ADS_CONV;
    if (HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDR, &reg_pointer, 1, 10) != HAL_OK) return -2.0f;
    if (HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDR, rx_data, 2, 10) != HAL_OK) return -3.0f;

    int16_t raw_adc = (int16_t)((rx_data[0] << 8) | rx_data[1]);
    if (raw_adc < 0) raw_adc = 0;

    float raw_adc_voltage = (float)raw_adc * 0.000125f;

    /* Returns uncalibrated op-amp scaled voltage to prevent double calibration */
    return raw_adc_voltage / OPAMP_GAIN_FACTOR;
}

/**
  * @brief  Extracts Shunt current and Bus voltage from raw INA registers.
  */
void INA_ReadData(void)
{
    uint8_t reg;
    uint8_t rx_buf[2];
    int16_t temp_raw;

    /* --- 1. Current Register Extraction & Precise Single-Pass Calibration --- */
    reg = 0x04;
    if (HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR, &reg, 1, 100) == HAL_OK)
    {
        if (HAL_I2C_Master_Receive(&hi2c1, INA226_ADDR, rx_buf, 2, 100) == HAL_OK)
        {
            temp_raw = (int16_t)((rx_buf[0] << 8) | rx_buf[1]);
            float raw_current = (float)temp_raw / 1000.0f;

            /* Direct Inline Inverse Calibration for Current */
            current_A = (raw_current - C_CURRENT) / M_CURRENT;

            /* Zero-current Noise Deadband Window (+/- 5mA clamp)
               Keeps full bi-directional tracking intact for battery diagnostics */
            if (current_A > -0.005f && current_A < 0.005f) {
                current_A = 0.0f;
            }
        }
    }

    /* --- 2. Bus Voltage Register Extraction (0x02) --- */
    reg = REG_BUS_VOLT;
    if (HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR, &reg, 1, 100) == HAL_OK)
    {
        if (HAL_I2C_Master_Receive(&hi2c1, INA226_ADDR, rx_buf, 2, 100) == HAL_OK)
        {
            temp_raw = (int16_t)((rx_buf[0] << 8) | rx_buf[1]);
            bus_voltage_V = (float)temp_raw * 0.00125f;
        }
    }
}

/**
  * @brief  Scans SD Card root sequentially to find and mount an available clean log session file.
  */
void CreateNewLogFile(void)
{
    Set_SPI_Speed(SPI_BAUDRATEPRESCALER_64);
    for (int i = 0; i < 1000; i++)
    {
        sprintf(current_filename, "LOG%03d.CSV", i);
        if (f_open(&fil, current_filename, FA_READ) != FR_OK)
        {
            if (f_open(&fil, current_filename, FA_CREATE_NEW | FA_WRITE) == FR_OK)
            {
                sprintf(buffer, "TIMESTAMP,CELL_v1,CELL_v2,CELL_v3,CELL_v4,SHUNT_CURR_A,BUS_VOLT_V\r\n");
                UINT bytesWritten;
                f_write(&fil, buffer, strlen(buffer), &bytesWritten);
                f_close(&fil);

                sprintf(buffer, "Created Session Log File Vector: %s\r\n", current_filename);
                HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
            }
            break;
        }
        else
        {
            f_close(&fil);
        }
    }
    Set_SPI_Speed(SPI_BAUDRATEPRESCALER_2);
}

void Custom_DrawFrameBorders(uint16_t color)
{
    ILI9488_FillRect(5, 5,   314, 2, color);
    ILI9488_FillRect(5, 470, 314, 2, color);
    ILI9488_FillRect(5,   5, 2, 465, color);
    ILI9488_FillRect(318, 5, 2, 465, color);
}

/**
  * @brief  Rapid non-blocking background sweep of active I2C addresses for LCD updates.
  */
void I2C_Scan_DisplayOnly(void)
{
    char temp[12];
    uint8_t node_match = 0;
    strcpy(i2c_display_line, "I2C:");

    for (uint8_t addr = 1; addr < 128; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 1, 2) == HAL_OK)
        {
            sprintf(temp, " 0x%02X", addr);
            strcat(i2c_display_line, temp);
            node_match++;

            if(node_match >= 5) {
                strcat(i2c_display_line, "..");
                break;
            }
        }
    }
    if (node_match == 0)
    {
        strcpy(i2c_display_line, "I2C Bus: NO NODES FOUND");
    }
}

/**
  * @brief  Comprehensive network topology mapping of active physical I2C targets on system boot.
  */
void I2C_Scan_All_Devices(void)
{
    uint8_t found = 0;
    char buf[64];

    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n--- I2C BUS SCAN ---\r\n", 23, 100);

    for (uint8_t addr = 1; addr < 128; addr++)
    {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (addr << 1), 1, 5) == HAL_OK)
        {
            found++;
            sprintf(buf, "\r\n  [+] Found Device at: 0x%02X\r\n", addr);
            HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 100);
        }
    }

    if (found == 0) {
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n  [-] No devices found.\r\n", 27, 100);
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n---------------------\r\n", 25, 100);
}

/**
  * @brief  Initializes INA226 Configuration settings.
  */
void INA_Setup(void)
{
    uint8_t data[3];

    data[0] = REG_CONFIG;
    data[1] = 0x45;
    data[2] = 0x67;
    HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR, data, 3, 100);

    data[0] = 0x05;
    data[1] = 0x0A;
    data[2] = 0x00;
    HAL_I2C_Master_Transmit(&hi2c1, INA226_ADDR, data, 3, 100);
}
/* USER CODE END 4 */

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10D19CE4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStructI2C = {0};

  GPIO_InitStructI2C.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStructI2C.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStructI2C.Pull = GPIO_PULLUP;
  GPIO_InitStructI2C.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructI2C.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStructI2C);

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(TFT_RST_GPIO_Port, TFT_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TFT_DC_GPIO_Port, TFT_DC_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TFT_BL_GPIO_Port, TFT_BL_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TFT_CS_GPIO_Port, TFT_CS_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Set initial state to HIGH (OFF) for safety interlock on system boot */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_0, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TFT_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TFT_RST_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TFT_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TFT_DC_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = TFT_BL_Pin|TFT_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
