#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <math.h>

#include <Wire.h>
#include <U8g2lib.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include <nn.h>

// build model with https://github.com/kraiskil/onnx2c
const int history_len = 15;
const int history_int = 2;
const int obs_len = 37;
const int action_len = 8;

float history[history_len * history_int][obs_len + action_len];
float current_obs[obs_len];
float actions[action_len];

float joint_vels[action_len];
float joint_pos[action_len];

float time_ref = 0.5;

float target_vel = 0.0f;
float target_horiz_vel = 0.0f;
float target_yaw_rate = 0.0f;

// Init MPU6050
TwoWire Wire2 = TwoWire(PB11, PB10); // For MPU6050
MPU6050 mpu(0x68, &Wire2);

// DMP Vars
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

static const float GYRO_TO_RAD_S = (1.0f / 16.384f) * (float)M_PI / 180.0f; // 0.00106526

// DMP orientation/motion vars
Quaternion q;        // [w, x, y, z]         Quaternion container
VectorInt16 aa;      // [x, y, z]            Accel sensor measurements
VectorInt16 gg;      // [x, y, z]            Gyro sensor measurements
VectorFloat gravity; // [x, y, z]            Gravity vector
float ypr[3];        // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

// Screens
U8G2_SSD1306_128X64_NONAME_F_HW_I2C l_screen(U8G2_R0);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C r_screen(U8G2_R0);

// Eye position offsets
int x = 0;
int y = 0;

// Init CAN
// STM32_CAN Can(PA11, PA12); // PA11/12
// static CAN_message_t TX_msgs[8];
// static CAN_message_t RX_msgs[8];

TaskHandle_t screenTask;
static void Screens(void *arg);
TaskHandle_t sensorTask;
static void SensorRead(void *arg);
TaskHandle_t modelTask;
static void Model(void *arg);
// TaskHandle_t commsTask;
// static void Comms(void *arg);

FDCAN_HandleTypeDef hfdcan1;
static const uint32_t CAN_ID_CMD = 0x100;     // mainboard -> all ESCs (broadcast)
static const uint32_t CAN_ID_FB_BASE = 0x200; // ESC n replies on 0x200 + n

/**
 * @brief FDCAN MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hfdcan: FDCAN handle pointer
 * @retval None
 */
extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if (hfdcan->Instance == FDCAN1)
  {
    /* USER CODE BEGIN FDCAN1_MspInit 0 */

    /* USER CODE END FDCAN1_MspInit 0 */

    /** Initializes the peripherals clock
     */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.PLL2.PLL2M = 4;
    PeriphClkInitStruct.PLL2.PLL2N = 9;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 3;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 3072.0;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PA11     ------> FDCAN1_RX
    PA12     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN FDCAN1_MspInit 1 */

    /* USER CODE END FDCAN1_MspInit 1 */
  }
}

/**
 * @brief FDCAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_NO_BRS;
  // hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 7;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 5;
  hfdcan1.Init.DataSyncJumpWidth = 2;
  hfdcan1.Init.DataTimeSeg1 = 7;
  hfdcan1.Init.DataTimeSeg2 = 2;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.RxFifo0ElmtsNbr = 24;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_64;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_64;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Serial.println("HAL FDCAN init failed");
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef f = {};
  f.IdType = FDCAN_STANDARD_ID;
  f.FilterIndex = 0;
  f.FilterType = FDCAN_FILTER_MASK;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1 = 0x200; // ID
  f.FilterID2 = 0x700; // mask: top 3 bits must match
  HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                               FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
  {
    Serial.println("HAL FDCAN start failed");
    Error_Handler();
  }
  /* USER CODE END FDCAN1_Init 2 */
}

static bool CAN_SendActions(const float *a) // 8 floats = 32 bytes, one FD frame
{
  FDCAN_TxHeaderTypeDef tx = {};
  tx.Identifier = CAN_ID_CMD;
  tx.IdType = FDCAN_STANDARD_ID;
  tx.TxFrameType = FDCAN_DATA_FRAME;
  tx.DataLength = FDCAN_DLC_BYTES_32;
  tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx.BitRateSwitch = FDCAN_BRS_OFF;
  tx.FDFormat = FDCAN_FD_CAN;
  tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx.MessageMarker = 0;

  uint8_t buf[32];
  memcpy(buf, a, sizeof(buf));
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, buf) == HAL_OK;
}

static void CAN_Poll()
{
  FDCAN_RxHeaderTypeDef rx;
  uint8_t buf[64];
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, buf) != HAL_OK)
      break;
    uint32_t idx = rx.Identifier - CAN_ID_FB_BASE;
    if (idx < 8)
    {
      memcpy(&joint_pos[idx], &buf[0], 4);
      memcpy(&joint_vels[idx], &buf[4], 4);
    }
  }
}

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Starting...");

  // Init arrays to all zeros
  for (int i = 0; i < history_len * history_int; i++)
  {
    for (int j = 0; j < obs_len + action_len; j++)
    {
      history[i][j] = 0.0f;
    }
  }

  for (int i = 0; i < obs_len; i++)
  {
    current_obs[i] = 0.0f;
  }

  for (int i = 0; i < action_len; i++)
  {
    actions[i] = 0.0f;
  }

  Wire.setSCL(PB6);
  Wire.setSDA(PB7);
  Wire.setClock(400000);
  Wire.begin();

  Wire2.setSCL(PB10);
  Wire2.setSDA(PB11);
  Wire2.setClock(400000);
  Wire2.begin();

  l_screen.setI2CAddress(0x78);
  r_screen.setI2CAddress(0x7A);

  l_screen.setBusClock(400000);
  r_screen.setBusClock(400000);

  l_screen.begin();
  r_screen.begin();

  mpu.initialize();
  Serial.println("Testing MPU6050 connection...");
  if (mpu.testConnection() == false)
  {
    Serial.println("MPU6050 connection failed");
    while (true)
      ;
  }
  else
  {
    Serial.println("MPU6050 connection successful");
  }

  devStatus = mpu.dmpInitialize();

  if (devStatus == 0)
  {
    mpu.CalibrateAccel(6); // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP...")); // Turning ON DMP
    mpu.setDMPEnabled(true);

    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison
  }

  HAL_FDCAN_MspInit(&hfdcan1);
  MX_FDCAN1_Init();

  xTaskCreate(Screens, NULL, 2048, NULL, 0, &screenTask);
  xTaskCreate(SensorRead, NULL, 2048, NULL, 0, &sensorTask);
  xTaskCreate(Model, NULL, 2048, NULL, 1, &modelTask);

  vTaskStartScheduler();
}

void drawStr(U8G2 screen, String str)
{
  screen.setFont(u8g2_font_6x13_tf);

  int line_len = 128 / 6;
  int num_lines = str.length() / line_len + 1;

  for (int i = 0; i < num_lines; i++)
  {
    screen.drawStr(0, (i + 1) * 13, str.substring(i * line_len, min((i + 1) * line_len, int(str.length()))).c_str());
  }
}

void drawEye(U8G2 *screen, float open_percent)
{
  int w = 44;
  int h = 31.0 * open_percent;

  int h_down = (1.0 - open_percent) * 32;

  screen->drawEllipse(max(min(64 + x, 127), 0), max(min(31 + y, 63), 0), w, h);
  screen->drawEllipse(max(min(64 + x, 127), 0), max(min(31 + y, 63), 0), w - 1, h - 1);

  screen->drawEllipse(max(min(64 + x, 127), 0), max(min(31 + y, 63), 0), w, h - 7);

  screen->drawCircle(max(min(64 + x, 127), 0), max(min(31 + y, 63), 0), min(9, h));

  screen->drawLine(max(min(64 + x, 127), 0), max(min(15 + h_down + y, 63), 0), max(min(64 + x, 127), 0), max(min(0 + h_down + y, 63), 0));

  screen->drawLine(max(min(74 + x, 127), 0), max(min(17 + h_down + y, 63), 0), max(min(76 + x, 127), 0), max(min(3 + h_down + y, 63), 0));
  screen->drawLine(max(min(54 + x, 127), 0), max(min(17 + h_down + y, 63), 0), max(min(52 + x, 127), 0), max(min(3 + h_down + y, 63), 0));

  screen->drawLine(max(min(44 + x, 127), 0), max(min(17 + h_down + y, 63), 0), max(min(42 + x, 127), 0), max(min(3 + h_down + y, 63), 0));
  screen->drawLine(max(min(84 + x, 127), 0), max(min(17 + h_down + y, 63), 0), max(min(86 + x, 127), 0), max(min(3 + h_down + y, 63), 0));

  screen->drawLine(max(min(34 + x, 127), 0), max(min(19 + h_down + y, 63), 0), max(min(32 + x, 127), 0), max(min(5 + h_down + y, 63), 0));
  screen->drawLine(max(min(94 + x, 127), 0), max(min(19 + h_down + y, 63), 0), max(min(96 + x, 127), 0), max(min(5 + h_down + y, 63), 0));

  if (millis() % 1000 < 500)
  {
    screen->drawCircle(0, 0, 10);
  }
}

static void Screens(void *arg)
{
  while (true)
  {
    l_screen.clearBuffer();
    drawEye(&l_screen, 1.0 - 0.9 * (rand() < __INT_MAX__ / 8));
    l_screen.sendBuffer();
    vTaskDelay(pdMS_TO_TICKS(10));

    r_screen.clearBuffer();
    drawEye(&r_screen, 1.0 - 0.9 * (rand() < __INT_MAX__ / 8));
    r_screen.sendBuffer();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static void SensorRead(void *arg)
{
  while (true)
  {
    if (!DMPReady)
      continue;

    /* Read a packet from FIFO */
    if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer))
    {
      // Get the Latest packet
      mpu.dmpGetQuaternion(&q, FIFOBuffer);

      mpu.dmpGetGravity(&gravity, &q); // gravity vector
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      mpu.dmpGetGyro(&gg, FIFOBuffer);

      x = int(ypr[2] * 64.0 / 3.14159);
      y = int(ypr[1] * 64.0 / 3.14159);
    }
  }
}

// static void Comms(void *arg)
// {
//   while (true)
//   {
//   }
// }

static void Model(void *arg)
{
  while (true)
  {
    uint32_t t1 = micros();

    CAN_SendActions(actions);
    vTaskDelay(pdMS_TO_TICKS(2));
    CAN_Poll();

    time_ref = fmodf(millis() * 0.003f, 1.0f); // 3 Hz gait

    // idxs in order: [r_side_hip, l_side_hip, r_hip, l_hip, r_knee, l_knee, r_ankle, l_ankle]

    // obs data (float[37]):

    // joint vel (rad/s?) l_side_hip, l_hip, l_knee, l_ankle
    // joint vel (rad/s?) r_side_hip, r_hip, r_knee, r_ankle
    for (int i = 0; i < 8; i++)
    {
      current_obs[i] = joint_vels[i];
    }

    // joint pos (rad) l_side_hip, l_hip, l_knee, l_ankle
    // joint pos (rad) r_side_hip, r_hip, r_knee, r_ankle
    // joint pos is in sin, cos form:
    // sin(l_side_hip), cos(l_side_hip), sin(l_hip), etc...
    for (int i = 0; i < 8; i++)
    {
      current_obs[8 + i * 2] = sin(joint_pos[i]);
      current_obs[8 + i * 2 + 1] = cos(joint_pos[i]);
    }

    // timing reference sin cos form
    // tl and tr are numbers between 1 and 0 for each side's phase
    // sin(2pi * tl), cos(2pi * tl), sin(2pi * tr), cos(2pi * tr)
    current_obs[24] = sin(2.0 * M_PI * time_ref);
    current_obs[25] = cos(2.0 * M_PI * time_ref);
    current_obs[26] = sin(2.0 * M_PI * (time_ref + 0.5));
    current_obs[27] = cos(2.0 * M_PI * (time_ref + 0.5));

    // gravity vector
    // -gravity.x, -gravity.y, -gravity.z
    current_obs[28] = -gravity.x;
    current_obs[29] = -gravity.y;
    current_obs[30] = -gravity.z;

    // ang vel
    // gg.x * GYRO_TO_RAD_S, gg.y * GYRO_TO_RAD_S, gg.z * GYRO_TO_RAD_S
    current_obs[31] = gg.x * GYRO_TO_RAD_S;
    current_obs[32] = gg.y * GYRO_TO_RAD_S;
    current_obs[33] = gg.z * GYRO_TO_RAD_S;

    // target_vel (m/s forwards/backwards, might be negative compared to intuitive)
    current_obs[34] = target_vel;
    // target_horiz_vel (m/s left/right)
    current_obs[35] = target_horiz_vel;
    // target_yaw_rate (rad/s)
    current_obs[36] = target_yaw_rate;

    ////// history:
    static float input[1][712];

    // the entire input is [current obs, history]
    // history is an array of last # [obs + actions]
    // right now, 15 history steps (but every other step), so history is float[15 * 2 * (37 + 8)]

    // obs is [the above obs, then history]
    for (int i = 0; i < obs_len; i++)
    {
      input[0][i] = current_obs[i];
    }

    for (int i = 0; i < history_len; i++)
    {
      for (int j = 0; j < obs_len + action_len; j++)
      {
        input[0][obs_len + i * (obs_len + action_len) + j] = history[i * history_int][j];
      }
    }

    // rolling works by shifting entire history right 1 and then adding new history to the front (0)
    // you do this after you create your obs array

    for (int i = history_len * history_int - 1; i > 0; i--)
    {
      for (int j = 0; j < obs_len + action_len; j++)
      {
        history[i][j] = history[i - 1][j];
      }
    }

    for (int i = 0; i < obs_len + action_len; i++)
    {
      history[0][i] = (i < obs_len) ? current_obs[i] : actions[i - obs_len];
    }

    float output[1][8];
    // order: [l_side_hip, l_hip, l_knee, l_ankle, r_side_hip, r_hip, r_knee, r_ankle]
    // multiplied by action_range / 2.0 + action_midpoint
    // action_range = upper limit - lower limit
    // action_midpoint = (upper limit + lower limit) / 2.0

    // Serial.println("Running model...");

    entry(input, output);
    uint32_t t2 = micros();
    // Serial.println("Time to run model: ");
    // Serial.println(t2 - t1);

    // target ~50it/s
    uint32_t remaining_time = 20000 - (t2 - t1);
    if ((t2 - t1) > 18000)
    {
      remaining_time = 2000;
    }

    for (int i = 0; i < action_len; i++)
    {
      actions[i] = output[0][i];
    }

    // Serial.println(remaining_time);

    vTaskDelay(pdMS_TO_TICKS(remaining_time / 1000));
  }
}

void loop()
{
  // unreachable once vTaskStartScheduler() is called in setup() - freertos
  // owns the cpu from that point on, so anything that needs to repeat has
  // to live in its own task (see Screens and SensorRead above).
}