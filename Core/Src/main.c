/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "bluetooth_control.h"
#include "lidar_pipeline.h"
#include "mapping_grid.h"
#include "motor_control.h"
#include "mpu6500.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OLED_I2C_ADDR               (0x3CU << 1)
#define OLED_WIDTH                  128U
#define OLED_HEIGHT                 64U
#define OLED_PAGE_COUNT             (OLED_HEIGHT / 8U)
#define OLED_FB_SIZE                (OLED_WIDTH * OLED_PAGE_COUNT)
#define OLED_I2C_TIMEOUT_MS         20U
#define OLED_RECOVER_RETRY_MS       1000U

#define ADC_CHANNEL_COUNT           3U
#define BUTTON_DEBOUNCE_MS          30U
#define ADC_PWM_MIN_PERMILLE        0U
#define ADC_PWM_MAX_PERMILLE        1000U
#define ADC_PWM_UPDATE_DEADBAND     8U

#define MAPPING_POINT_BATCH_LIMIT   128U
#define MAP_ROW_TX_INTERVAL_MS      60U
#define MAP_STAT_TX_INTERVAL_MS     2000U
#define POSE_TX_INTERVAL_MS         100U
#define LIDAR_DEBUG_MAX_TX_PER_BATCH 4U
#define ODOM_DEBUG_TX_INTERVAL_MS   500U
#define OBSTACLE_FRONT_SECTOR_CDEG 1500U
#define OBSTACLE_MIN_SAFE_MM       150U
#define OBSTACLE_MAX_SAFE_MM       800U
#define NAV_DRIVE_PWM_CAP          450U
#define ANGLE_TURN_PWM_CAP         520U
#define ANGLE_TURN_DEFAULT_DEG      90U
#define ANGLE_TURN_MIN_DEG          1U
#define ANGLE_TURN_MAX_DEG          360U
#define ANGLE_TURN_DONE_TOL_CDEG    250L
#define ANGLE_TURN_CORRECTION_TOL_CDEG 300L
#define ANGLE_TURN_MAX_CORRECTIONS  0U
#define ANGLE_TURN_TIMEOUT_MS       12000U
#define ANGLE_TURN_SLOW_ZONE_CDEG   3000L
#define ANGLE_TURN_FINE_ZONE_CDEG   1200L
#define ANGLE_TURN_SLOW_PWM         450U
#define ANGLE_TURN_FINE_PWM         320U
#define MAPPING_START_X_MM          (-1500L)
#define MAPPING_START_Y_MM          (-1500L)
#define MAPPING_START_HEADING_CDEG  0L
#define MAPPING_POSE_HISTORY_LENGTH 64U
#define MAPPING_LIDAR_POINT_MAX_AGE_MS 250U
#define MAZE_GRID_CELLS             5U
#define MAZE_CELL_SIZE_MM           700L
#define MAZE_ORIGIN_X_MM            (MAPPING_START_X_MM - (MAZE_CELL_SIZE_MM / 2L))
#define MAZE_ORIGIN_Y_MM            (MAPPING_START_Y_MM - (MAZE_CELL_SIZE_MM / 2L))
#define MAZE_TARGET_TOLERANCE_MM    140L
#define NAV_TURN_HEADING_TOL_CDEG   900L
#define NAV_DRIVE_HEADING_TOL_CDEG  36000L
#define NAV_TURN_MAX_RETRIES        0U
#define NAV_TURN_BRAKE_SETTLE_MS    260U
#define NAV_DRIVE_BRAKE_SETTLE_MS   180U
#define NAV_OBSTACLE_HOLD_MS        250U
#define NAV_OBSTACLE_STOP_MARGIN_MM 50U
#define NAV_OBSTACLE_MIN_CLUSTER_POINTS 3U
#define NAV_OBSTACLE_CLUSTER_ANGLE_CDEG 700U
#define NAV_OBSTACLE_CLUSTER_RANGE_MM  260U
#define NAV_OBSTACLE_CONFIRM_MS     450U
#define NAV_TARGET_LOOKAHEAD_CELLS  2U
#define MAPPING_SCAN_BUFFER_POINTS  360U
#define MAPPING_SCAN_MIN_POINTS     12U
#define MAPPING_SCAN_MIN_CLUSTER_POINTS 2U
#define MAPPING_SCAN_CLUSTER_LOOKAHEAD 2U
#define MAPPING_SCAN_CLUSTER_ANGLE_CDEG 700U
#define MAPPING_SCAN_CLUSTER_RANGE_MM 280U
#define MAPPING_SCAN_SCORE_STRIDE   6U
#define MAPPING_SCAN_DEFAULT_PERIOD_MS 180U
#define MAPPING_SCAN_MIN_PERIOD_MS  80U
#define MAPPING_SCAN_MAX_PERIOD_MS  350U
#define MAPPING_POINT_MIN_QUALITY   1U
#define MAPPING_MOTION_GYRO_HOLD_DPS_X100 4500L
#define MAPPING_MATCH_MIN_OCCUPIED_CELLS 8U
#define MAPPING_MATCH_MIN_USED_POINTS 8U
#define MAPPING_MATCH_MIN_SCORE     8L
#define MAPPING_MATCH_XY_RANGE_MM   100L
#define MAPPING_MATCH_XY_STEP_MM    50L
#define MAPPING_MATCH_HEADING_RANGE_CDEG 500L
#define MAPPING_MATCH_HEADING_STEP_CDEG 100L
#define ENCODER_RIGHT_DELTA_SIGN    (-1L)
#define MAPPING_ENCODER_MM_PER_COUNT_X1000 133L
#define GYRO_STATIONARY_COUNT_THRESHOLD 2L
#define GYRO_BIAS_FILTER_DIVISOR    16L
#define GYRO_DEADBAND_DPS_X100      15L
#define MAPPING_PI                  3.14159265358979323846f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
osThreadId senseTaskHandle;
osThreadId uiTaskHandle;
osThreadId btTaskHandle;

typedef enum
{
  TEST_PAGE_LIDAR = 0,
  TEST_PAGE_MPU,
  TEST_PAGE_ADC,
  TEST_PAGE_ENCODER,
  TEST_PAGE_COUNT
} test_page_t;

typedef struct
{
  uint16_t last_left_counter;
  uint16_t last_right_counter;
  int16_t left_delta;
  int16_t right_delta;
  int32_t left_total;
  int32_t right_total;
} encoder_test_state_t;

typedef enum
{
  NAV_STATE_IDLE = 0,
  NAV_STATE_TURNING,
  NAV_STATE_DRIVING,
  NAV_STATE_SETTLING,
  NAV_STATE_DONE,
  NAV_STATE_FAILED
} nav_state_t;

typedef enum
{
  NAV_SETTLE_NEXT_NONE = 0,
  NAV_SETTLE_NEXT_DRIVE,
  NAV_SETTLE_NEXT_LEG,
  NAV_SETTLE_NEXT_TURN
} nav_settle_next_t;

typedef struct
{
  uint8_t x;
  uint8_t y;
} maze_cell_t;

typedef struct
{
  uint32_t tick_ms;
  MappingGridPose_t pose;
  uint16_t angle_cdeg;
  uint16_t distance_mm;
  uint8_t quality;
} mapping_scan_point_t;

typedef struct
{
  uint32_t scans_started;
  uint32_t scans_completed;
  uint32_t scans_inserted;
  uint32_t scans_matched;
  uint32_t scans_unmatched;
  uint32_t scans_motion_held;
  uint32_t scans_short;
  uint32_t points_buffered;
  uint32_t points_inserted;
  uint32_t points_quality_rejected;
  uint32_t points_overflowed;
  uint32_t pose_misses;
  uint16_t last_scan_points;
  uint16_t last_scan_inserted;
  uint16_t last_match_used;
  int32_t last_match_score;
  int32_t last_match_dx_mm;
  int32_t last_match_dy_mm;
  int32_t last_match_dtheta_cdeg;
} mapping_diagnostics_t;

static uint16_t adc_buf[ADC_CHANNEL_COUNT];
static uint8_t oled_fb[OLED_FB_SIZE];
static uint8_t oled_page_tx[OLED_WIDTH + 1U];

static volatile bool app_ready = false;
static bool oled_ready = false;
static test_page_t current_page = TEST_PAGE_LIDAR;
static uint8_t button_stable_mask = 0U;
static uint8_t button_prev_nonzero = 0U;
static uint8_t button_sample_mask = 0U;
static uint32_t button_change_tick_ms = 0U;
static uint32_t last_lidar_result_tick_ms = 0U;
static uint32_t last_oled_tick_ms = 0U;
static uint32_t last_oled_recover_tick_ms = 0U;
static uint32_t last_sensor_tick_ms = 0U;
static uint32_t last_map_row_tx_tick_ms = 0U;
static uint32_t last_map_stat_tx_tick_ms = 0U;
static uint32_t last_pose_tx_tick_ms = 0U;
static uint32_t last_mapping_pose_tick_ms = 0U;
static uint32_t mapping_start_tick_ms = 0U;

static bool lidar_result_valid = false;
static bool mapping_active = false;
static bool lidar_debug_active = false;
static bool odom_debug_active = false;
static LidarParseResult_t lidar_result = {0};
static Mpu6500State_t mpu_state = {0};
static encoder_test_state_t encoder_test = {0};
static MappingGridPose_t mapping_pose = {0};
static mapping_scan_point_t mapping_scan_points[MAPPING_SCAN_BUFFER_POINTS];
static mapping_diagnostics_t mapping_diag = {0};
static uint8_t next_map_tx_row = 0U;
static uint32_t lidar_debug_seq = 0U;
static int32_t mapping_travel_residual_x1000 = 0L;
static uint32_t mapping_scan_start_tick_ms = 0U;
static uint16_t mapping_scan_start_angle_cdeg = 0U;
static uint16_t mapping_scan_count = 0U;
static bool mapping_scan_open = false;
static bool mapping_scan_motion_blocked = false;
static uint32_t last_odom_debug_update_tick_ms = 0U;
static uint32_t last_odom_debug_tx_tick_ms = 0U;
static uint32_t odom_debug_seq = 0U;
static int32_t odom_debug_left_counts = 0L;
static int32_t odom_debug_right_counts = 0L;
static int32_t odom_debug_heading_cdeg = 0L;
static int32_t gyro_z_bias_dps_x100 = 0L;
static int32_t gyro_z_corrected_dps_x100 = 0L;
static MappingGridPose_t mapping_pose_history[MAPPING_POSE_HISTORY_LENGTH];
static uint32_t mapping_pose_history_ticks[MAPPING_POSE_HISTORY_LENGTH];
static uint8_t mapping_pose_history_next = 0U;
static uint8_t mapping_pose_history_count = 0U;
static bool angle_turn_active = false;
static int8_t angle_turn_direction = 0;
static bool angle_turn_heading_target_valid = false;
static uint8_t angle_turn_correction_count = 0U;
static int32_t angle_turn_target_cdeg = 0L;
static int32_t angle_turn_target_heading_cdeg = 0L;
static int32_t angle_turn_progress_cdeg = 0L;
static uint32_t angle_turn_last_tick_ms = 0U;
static uint32_t angle_turn_start_tick_ms = 0U;
static maze_cell_t nav_start_cell = {0U, 0U};
static maze_cell_t nav_goal_cell = {4U, 4U};
static maze_cell_t nav_path[MAZE_GRID_CELLS * MAZE_GRID_CELLS];
static uint8_t nav_path_length = 0U;
static uint8_t nav_path_index = 0U;
static uint8_t nav_turn_retry_count = 0U;
static nav_state_t nav_state = NAV_STATE_IDLE;
static nav_settle_next_t nav_settle_next = NAV_SETTLE_NEXT_NONE;
static bool nav_active = false;
static uint32_t nav_settle_until_ms = 0U;
static int32_t nav_target_x_mm = 0L;
static int32_t nav_target_y_mm = 0L;
static int32_t nav_target_heading_cdeg = 0L;
static bool nav_blocked_vertical[MAZE_GRID_CELLS + 1U][MAZE_GRID_CELLS];
static bool nav_blocked_horizontal[MAZE_GRID_CELLS][MAZE_GRID_CELLS + 1U];
static uint16_t nav_front_min_mm = UINT16_MAX;
static uint16_t nav_front_blocking_angle_cdeg = 0U;
static uint32_t nav_front_update_tick_ms = 0U;
static uint16_t nav_front_scan_min_mm = UINT16_MAX;
static uint16_t nav_front_scan_angle_cdeg = 0U;
static uint16_t nav_front_last_angle_cdeg = 0U;
static uint16_t nav_front_last_distance_mm = 0U;
static uint8_t nav_front_cluster_count = 0U;
static bool nav_front_cluster_open = false;
static bool nav_block_candidate_active = false;
static uint32_t nav_block_candidate_start_ms = 0U;
static uint16_t nav_block_candidate_distance_mm = UINT16_MAX;
static uint16_t nav_block_candidate_angle_cdeg = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART6_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
static void TestApp_InitPeripherals(void);
static void TestApp_PollButtons(void);
static void TestApp_UpdateSensors(void);
static void TestApp_UpdateDisplay(void);
static void TestApp_HandleBluetoothCommands(void);
static void TestApp_UpdateMotorSpeedFromAdc(void);
static void TestApp_StartMapping(void);
static void TestApp_StopMapping(void);
static void TestApp_ClearMappingData(void);
static void TestApp_ProcessLidarPoints(void);
static void TestApp_SendLidarDebugPoint(const LidarPoint_t *point);
static void TestApp_ResetMappingScanState(void);
static void TestApp_ResetMappingDiagnostics(void);
static void TestApp_HandleMappingScanPoint(const LidarPoint_t *point, bool point_is_fresh);
static void TestApp_StartMappingScan(const LidarPoint_t *point);
static void TestApp_BufferMappingScanPoint(const LidarPoint_t *point);
static void TestApp_ProcessCompletedMappingScan(uint32_t end_tick_ms);
static bool TestApp_PrepareMappingScan(uint32_t end_tick_ms, uint16_t *out_valid_points);
static bool TestApp_ShouldHoldMappingUpdates(void);
static bool TestApp_MatchMappingScan(const MappingGridPose_t *base_end_pose,
                                     MappingGridPose_t *out_matched_end_pose,
                                     int32_t *out_score,
                                     uint16_t *out_used_points);
static int32_t TestApp_ScoreMappingScanCandidate(const MappingGridPose_t *base_end_pose,
                                                 const MappingGridPose_t *candidate_end_pose,
                                                 uint16_t *out_used_points);
static MappingGridPose_t TestApp_ApplyScanPoseCorrection(const MappingGridPose_t *point_pose,
                                                         const MappingGridPose_t *base_end_pose,
                                                         const MappingGridPose_t *candidate_end_pose);
static uint16_t TestApp_InsertPreparedMappingScan(const MappingGridPose_t *base_end_pose,
                                                  const MappingGridPose_t *matched_end_pose);
static uint32_t TestApp_EstimateScanPointTick(uint16_t angle_cdeg, uint32_t scan_period_ms);
static void TestApp_UpdateMappingPose(uint32_t now_ms, int16_t left_delta, int16_t right_delta);
static void TestApp_ResetPoseHistory(void);
static void TestApp_RecordPoseHistory(uint32_t tick_ms);
static bool TestApp_GetPoseForTick(uint32_t tick_ms, MappingGridPose_t *out_pose);
static uint16_t TestApp_AngleDeltaAbsCdeg(uint16_t a_cdeg, uint16_t b_cdeg);
static bool TestApp_AreScanPointsContinuous(const mapping_scan_point_t *a, const mapping_scan_point_t *b);
static uint16_t TestApp_FilterMappingScanOutliers(void);
static void TestApp_StartOdomDebug(void);
static void TestApp_StopOdomDebug(void);
static void TestApp_UpdateGyroDriftCompensation(int16_t left_delta, int16_t right_delta);
static void TestApp_UpdateOdomDebug(uint32_t now_ms, int16_t left_delta, int16_t right_delta);
static void TestApp_StreamOdomDebug(void);
static bool TestApp_IsFrontLidarPoint(uint16_t angle_cdeg);
static void TestApp_StartAngleTurn(int8_t direction, uint16_t degrees);
static void TestApp_StartHeadingTurn(int32_t target_heading_cdeg, uint8_t correction_count, const char *reason);
static void TestApp_StopAngleTurn(bool completed);
static void TestApp_UpdateAngleTurn(uint32_t now_ms);
static bool TestApp_ParseMazeCell(const char *text, maze_cell_t *out_cell);
static bool TestApp_IsMazeCellValid(const maze_cell_t *cell);
static int32_t TestApp_MazeCellCenterX(uint8_t x);
static int32_t TestApp_MazeCellCenterY(uint8_t y);
static uint16_t TestApp_MazeCellManhattan(const maze_cell_t *a, const maze_cell_t *b);
static void TestApp_SetPoseToMazeCell(const maze_cell_t *cell);
static maze_cell_t TestApp_GetNearestMazeCell(void);
static void TestApp_ResetNavObstacle(void);
static void TestApp_UpdateNavObstacle(const LidarPoint_t *point, uint32_t now_ms);
static void TestApp_CommitNavFrontCluster(uint32_t now_ms);
static bool TestApp_HasFreshNavFrontSample(uint32_t now_ms);
static bool TestApp_IsNavFrontBlocked(uint32_t now_ms, uint16_t *out_distance_mm, uint16_t *out_angle_cdeg);
static bool TestApp_ConfirmNavBlocked(uint32_t now_ms, uint16_t distance_mm, uint16_t angle_cdeg);
static void TestApp_ResetNavBlockCandidate(void);
static void TestApp_ClearNavDynamicWalls(void);
static void TestApp_MarkNavBlockedEdge(void);
static bool TestApp_ReplanNavigationFromPose(const char *reason);
static void TestApp_UpdateNavTargetHeading(void);
static void TestApp_StartNavigation(void);
static void TestApp_StopNavigation(const char *reason);
static void TestApp_UpdateNavigation(uint32_t now_ms);
static void TestApp_StartNavSettle(uint32_t now_ms, uint32_t duration_ms, nav_settle_next_t next_action);
static bool TestApp_PlanPathAStar(const maze_cell_t *start_cell, const maze_cell_t *goal_cell);
static void TestApp_BuildMazeBounds(bool vertical[MAZE_GRID_CELLS + 1U][MAZE_GRID_CELLS],
                                    bool horizontal[MAZE_GRID_CELLS][MAZE_GRID_CELLS + 1U]);
static bool TestApp_IsWallBetween(const bool vertical[MAZE_GRID_CELLS + 1U][MAZE_GRID_CELLS],
                                  const bool horizontal[MAZE_GRID_CELLS][MAZE_GRID_CELLS + 1U],
                                  uint8_t x,
                                  uint8_t y,
                                  int8_t dx,
                                  int8_t dy);
static bool TestApp_SearchPathAStar(const maze_cell_t *start_cell,
                                    const maze_cell_t *goal_cell,
                                    const bool vertical[MAZE_GRID_CELLS + 1U][MAZE_GRID_CELLS],
                                    const bool horizontal[MAZE_GRID_CELLS][MAZE_GRID_CELLS + 1U]);
static void TestApp_StartNavLeg(void);
static uint16_t TestApp_ParseTurnDegrees(const char *text);
static int32_t TestApp_SignedHeadingErrorCdeg(int32_t target_cdeg, int32_t current_cdeg);
static void TestApp_StreamMap(void);
static void TestApp_StreamPose(void);
static const char *TestApp_GetMotionStateText(void);
static void TestApp_RequestFullMapStream(void);
static void TestApp_SendMapHeader(const char *state);
static void TestApp_SendMapStat(void);
static void TestApp_SendMapDiag(void);
static int32_t AppAbs32(int32_t value);
static uint16_t ClampPwmPermille(uint16_t value, uint16_t max_value);
static uint16_t GetObstacleSafeDistanceMm(void);

static bool OLED_InitMinimal(void);
static bool OLED_WriteCommand(uint8_t command);
static bool OLED_WriteCommands(const uint8_t *commands, uint16_t length);
static void OLED_Clear(void);
static void OLED_Update(void);
static void OLED_DrawChar6x8(uint8_t x, uint8_t page, char c);
static void OLED_DrawString6x8(uint8_t x, uint8_t page, const char *text);

static int16_t Encoder_ComputeDelta(uint16_t current, uint16_t previous);
static uint8_t Buttons_ReadMask(void);
static void RenderPage_LiDAR(void);
static void RenderPage_MPU(void);
static void RenderPage_ADC(void);
static void RenderPage_Encoder(void);
static void DrawStatusLine(uint8_t page, const char *label, int32_t value);
static uint16_t AdcToPwmPermille(uint16_t adc_value);
static uint16_t GetDrivePwmPermille(void);
static uint16_t GetTurnPwmPermille(void);
static int32_t NormalizeHeadingCdeg(int32_t heading_cdeg);

void StartSenseTask(void const * argument);
void StartUiTask(void const * argument);
void StartBtTask(void const * argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const uint8_t font_space[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t font_dash[5]  = {0x08, 0x08, 0x08, 0x08, 0x08};
static const uint8_t font_dot[5]   = {0x00, 0x60, 0x60, 0x00, 0x00};
static const uint8_t font_colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
static const uint8_t font_slash[5] = {0x20, 0x10, 0x08, 0x04, 0x02};
static const uint8_t font_equal[5] = {0x14, 0x14, 0x14, 0x14, 0x14};
static const uint8_t font_0[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E};
static const uint8_t font_1[5] = {0x00, 0x42, 0x7F, 0x40, 0x00};
static const uint8_t font_2[5] = {0x42, 0x61, 0x51, 0x49, 0x46};
static const uint8_t font_3[5] = {0x21, 0x41, 0x45, 0x4B, 0x31};
static const uint8_t font_4[5] = {0x18, 0x14, 0x12, 0x7F, 0x10};
static const uint8_t font_5[5] = {0x27, 0x45, 0x45, 0x45, 0x39};
static const uint8_t font_6[5] = {0x3C, 0x4A, 0x49, 0x49, 0x30};
static const uint8_t font_7[5] = {0x01, 0x71, 0x09, 0x05, 0x03};
static const uint8_t font_8[5] = {0x36, 0x49, 0x49, 0x49, 0x36};
static const uint8_t font_9[5] = {0x06, 0x49, 0x49, 0x29, 0x1E};
static const uint8_t font_A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
static const uint8_t font_B[5] = {0x7F, 0x49, 0x49, 0x49, 0x36};
static const uint8_t font_C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
static const uint8_t font_D[5] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
static const uint8_t font_E[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
static const uint8_t font_F[5] = {0x7F, 0x09, 0x09, 0x09, 0x01};
static const uint8_t font_G[5] = {0x3E, 0x41, 0x49, 0x49, 0x7A};
static const uint8_t font_H[5] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
static const uint8_t font_I[5] = {0x00, 0x41, 0x7F, 0x41, 0x00};
static const uint8_t font_J[5] = {0x20, 0x40, 0x41, 0x3F, 0x01};
static const uint8_t font_K[5] = {0x7F, 0x08, 0x14, 0x22, 0x41};
static const uint8_t font_L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
static const uint8_t font_M[5] = {0x7F, 0x02, 0x0C, 0x02, 0x7F};
static const uint8_t font_N[5] = {0x7F, 0x04, 0x08, 0x10, 0x7F};
static const uint8_t font_O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
static const uint8_t font_P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
static const uint8_t font_Q[5] = {0x3E, 0x41, 0x51, 0x21, 0x5E};
static const uint8_t font_R[5] = {0x7F, 0x09, 0x19, 0x29, 0x46};
static const uint8_t font_S[5] = {0x46, 0x49, 0x49, 0x49, 0x31};
static const uint8_t font_T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
static const uint8_t font_U[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F};
static const uint8_t font_V[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F};
static const uint8_t font_W[5] = {0x7F, 0x20, 0x18, 0x20, 0x7F};
static const uint8_t font_X[5] = {0x63, 0x14, 0x08, 0x14, 0x63};
static const uint8_t font_Y[5] = {0x03, 0x04, 0x78, 0x04, 0x03};
static const uint8_t font_Z[5] = {0x61, 0x51, 0x49, 0x45, 0x43};

static const uint8_t *OLED_GetGlyph(char c)
{
  switch (c)
  {
    case ' ': return font_space;
    case '-': return font_dash;
    case '.': return font_dot;
    case ':': return font_colon;
    case '/': return font_slash;
    case '=': return font_equal;
    case '0': return font_0;
    case '1': return font_1;
    case '2': return font_2;
    case '3': return font_3;
    case '4': return font_4;
    case '5': return font_5;
    case '6': return font_6;
    case '7': return font_7;
    case '8': return font_8;
    case '9': return font_9;
    case 'A': return font_A;
    case 'B': return font_B;
    case 'C': return font_C;
    case 'D': return font_D;
    case 'E': return font_E;
    case 'F': return font_F;
    case 'G': return font_G;
    case 'H': return font_H;
    case 'I': return font_I;
    case 'J': return font_J;
    case 'K': return font_K;
    case 'L': return font_L;
    case 'M': return font_M;
    case 'N': return font_N;
    case 'O': return font_O;
    case 'P': return font_P;
    case 'Q': return font_Q;
    case 'R': return font_R;
    case 'S': return font_S;
    case 'T': return font_T;
    case 'U': return font_U;
    case 'V': return font_V;
    case 'W': return font_W;
    case 'X': return font_X;
    case 'Y': return font_Y;
    case 'Z': return font_Z;
    default:  return font_space;
  }
}

static bool OLED_WriteCommand(uint8_t command)
{
  uint8_t payload[2] = {0x00U, command};
  return (HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, payload, sizeof(payload), OLED_I2C_TIMEOUT_MS) == HAL_OK);
}

static bool OLED_WriteCommands(const uint8_t *commands, uint16_t length)
{
  uint16_t i;

  for (i = 0U; i < length; ++i)
  {
    if (!OLED_WriteCommand(commands[i]))
    {
      return false;
    }
  }

  return true;
}

static bool OLED_InitMinimal(void)
{
  static const uint8_t init_cmds[] =
  {
    0xAEU, 0xD5U, 0x80U, 0xA8U, 0x3FU, 0xD3U, 0x00U, 0x40U,
    0x8DU, 0x14U, 0x20U, 0x00U, 0xA1U, 0xC8U, 0xDAU, 0x12U,
    0x81U, 0xCFU, 0xD9U, 0xF1U, 0xDBU, 0x40U, 0xA4U, 0xA6U,
    0xAFU
  };

  if (HAL_I2C_IsDeviceReady(&hi2c2, OLED_I2C_ADDR, 3U, OLED_I2C_TIMEOUT_MS) != HAL_OK)
  {
    return false;
  }

  if (!OLED_WriteCommands(init_cmds, sizeof(init_cmds)))
  {
    return false;
  }

  OLED_Clear();
  OLED_Update();
  return true;
}

static void OLED_Clear(void)
{
  memset(oled_fb, 0, sizeof(oled_fb));
}

static void OLED_DrawChar6x8(uint8_t x, uint8_t page, char c)
{
  const uint8_t *glyph;
  uint16_t offset;
  uint8_t col;

  if ((x > (OLED_WIDTH - 6U)) || (page >= OLED_PAGE_COUNT))
  {
    return;
  }

  glyph = OLED_GetGlyph(c);
  offset = ((uint16_t)page * OLED_WIDTH) + x;

  for (col = 0U; col < 5U; ++col)
  {
    oled_fb[offset + col] = glyph[col];
  }
  oled_fb[offset + 5U] = 0x00U;
}

static void OLED_DrawString6x8(uint8_t x, uint8_t page, const char *text)
{
  while ((*text != '\0') && (x <= (OLED_WIDTH - 6U)))
  {
    OLED_DrawChar6x8(x, page, *text);
    x = (uint8_t)(x + 6U);
    ++text;
  }
}

static void OLED_Update(void)
{
  uint8_t page;
  uint8_t commands[3];

  if (!oled_ready)
  {
    return;
  }

  oled_page_tx[0] = 0x40U;

  for (page = 0U; page < OLED_PAGE_COUNT; ++page)
  {
    commands[0] = (uint8_t)(0xB0U + page);
    commands[1] = 0x00U;
    commands[2] = 0x10U;

    if (!OLED_WriteCommands(commands, sizeof(commands)))
    {
      oled_ready = false;
      return;
    }

    memcpy(&oled_page_tx[1], &oled_fb[page * OLED_WIDTH], OLED_WIDTH);
    if (HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, oled_page_tx, sizeof(oled_page_tx), OLED_I2C_TIMEOUT_MS) != HAL_OK)
    {
      oled_ready = false;
      return;
    }
  }
}

static int16_t Encoder_ComputeDelta(uint16_t current, uint16_t previous)
{
  int32_t delta = (int32_t)current - (int32_t)previous;

  if (delta > 32767L)
  {
    delta -= 65536L;
  }
  else if (delta < -32768L)
  {
    delta += 65536L;
  }

  return (int16_t)delta;
}

static uint8_t Buttons_ReadMask(void)
{
  uint8_t mask = 0U;

  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) { mask |= 0x01U; }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == GPIO_PIN_RESET) { mask |= 0x02U; }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == GPIO_PIN_RESET) { mask |= 0x04U; }
  if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == GPIO_PIN_RESET) { mask |= 0x08U; }

  return mask;
}

static void TestApp_PollButtons(void)
{
  uint8_t raw = Buttons_ReadMask();
  uint8_t page_button_pressed;
  uint32_t now = HAL_GetTick();

  if (raw != button_sample_mask)
  {
    button_sample_mask = raw;
    button_change_tick_ms = now;
  }

  if ((now - button_change_tick_ms) < BUTTON_DEBOUNCE_MS)
  {
    return;
  }

  if (button_stable_mask != button_sample_mask)
  {
    button_stable_mask = button_sample_mask;
    page_button_pressed = (uint8_t)(button_stable_mask & 0x08U);
    if ((page_button_pressed != 0U) && (button_prev_nonzero == 0U))
    {
      current_page = (test_page_t)((current_page + 1U) % TEST_PAGE_COUNT);
    }
    button_prev_nonzero = page_button_pressed;
  }
}

static void DrawStatusLine(uint8_t page, const char *label, int32_t value)
{
  char line[22];
  (void)snprintf(line, sizeof(line), "%-7s:%ld", label, (long)value);
  OLED_DrawString6x8(0U, page, line);
}

static uint16_t AdcToPwmPermille(uint16_t adc_value)
{
  uint32_t span = (uint32_t)(ADC_PWM_MAX_PERMILLE - ADC_PWM_MIN_PERMILLE);

  if (adc_value > 4095U)
  {
    adc_value = 4095U;
  }

  return (uint16_t)(ADC_PWM_MIN_PERMILLE + (((uint32_t)adc_value * span) / 4095U));
}

static uint16_t GetDrivePwmPermille(void)
{
  return AdcToPwmPermille(adc_buf[0]);
}

static uint16_t GetTurnPwmPermille(void)
{
  return AdcToPwmPermille(adc_buf[1]);
}

static uint16_t ClampPwmPermille(uint16_t value, uint16_t max_value)
{
  return (value > max_value) ? max_value : value;
}

static uint16_t GetObstacleSafeDistanceMm(void)
{
  uint32_t span = (uint32_t)(OBSTACLE_MAX_SAFE_MM - OBSTACLE_MIN_SAFE_MM);
  uint16_t adc_value = adc_buf[2];

  if (adc_value > 4095U)
  {
    adc_value = 4095U;
  }

  return (uint16_t)(OBSTACLE_MIN_SAFE_MM + (((uint32_t)adc_value * span) / 4095U));
}

static bool PwmValueChanged(uint16_t current, uint16_t target)
{
  return (current > target) ?
      ((current - target) > ADC_PWM_UPDATE_DEADBAND) :
      ((target - current) > ADC_PWM_UPDATE_DEADBAND);
}

static int32_t NormalizeHeadingCdeg(int32_t heading_cdeg)
{
  while (heading_cdeg < 0L)
  {
    heading_cdeg += 36000L;
  }

  while (heading_cdeg >= 36000L)
  {
    heading_cdeg -= 36000L;
  }

  return heading_cdeg;
}

static int32_t AppAbs32(int32_t value)
{
  return (value < 0L) ? -value : value;
}

static void TestApp_UpdateGyroDriftCompensation(int16_t left_delta, int16_t right_delta)
{
  int32_t raw_gyro;
  int32_t diff;
  int32_t bias_step;
  bool stationary;

  if (!mpu_state.ready)
  {
    gyro_z_corrected_dps_x100 = 0L;
    return;
  }

  raw_gyro = mpu_state.gyro_z_dps_x100;
  stationary = ((AppAbs32((int32_t)left_delta) + AppAbs32((int32_t)right_delta)) <= GYRO_STATIONARY_COUNT_THRESHOLD);

  if (stationary)
  {
    diff = raw_gyro - gyro_z_bias_dps_x100;
    bias_step = diff / GYRO_BIAS_FILTER_DIVISOR;
    if ((bias_step == 0L) && (diff != 0L))
    {
      bias_step = (diff > 0L) ? 1L : -1L;
    }
    gyro_z_bias_dps_x100 += bias_step;
  }

  gyro_z_corrected_dps_x100 = raw_gyro - gyro_z_bias_dps_x100;
  if (AppAbs32(gyro_z_corrected_dps_x100) <= GYRO_DEADBAND_DPS_X100)
  {
    gyro_z_corrected_dps_x100 = 0L;
  }
}

static void RenderPage_LiDAR(void)
{
  uint32_t now = HAL_GetTick();
  bool stream_alive = false;

  if (lidar_result_valid && ((now - last_lidar_result_tick_ms) < 1000U) && (lidar_result.valid_nodes > 0U))
  {
    stream_alive = true;
  }

  OLED_DrawString6x8(0U, 0U, "LIDAR TEST");
  OLED_DrawString6x8(0U, 1U, (lidar_result_valid && (lidar_result.descriptor_seen != 0U)) ? "DESC   : YES" : "DESC   : NO");
  OLED_DrawString6x8(0U, 2U, mapping_active ? "MAP    : ON" : (stream_alive ? "STREAM : OK" : "STREAM : NO"));
  DrawStatusLine(3U, "BYTES", (int32_t)lidar_result.total_bytes);
  DrawStatusLine(4U, "VALID", (int32_t)lidar_result.valid_nodes);
  DrawStatusLine(5U, "INVAL", (int32_t)lidar_result.invalid_nodes);
  DrawStatusLine(6U, "DIST", (int32_t)lidar_result.last_distance_mm);
  DrawStatusLine(7U, "QUAL", (int32_t)lidar_result.last_quality);
}

static void RenderPage_MPU(void)
{
  char line[22];

  OLED_DrawString6x8(0U, 0U, "MPU6500 TEST");
  OLED_DrawString6x8(0U, 1U, mpu_state.ready ? "READY  : YES" : "READY  : NO");
  (void)snprintf(line, sizeof(line), "WHOAMI : %02X", mpu_state.who_am_i);
  OLED_DrawString6x8(0U, 2U, line);
  DrawStatusLine(3U, "GYROZ", (int32_t)mpu_state.gyro_z_raw);
  DrawStatusLine(4U, "DPSX100", mpu_state.gyro_z_dps_x100);
  OLED_DrawString6x8(0U, 6U, "BTN ANY TO PAGE");
  OLED_DrawString6x8(0U, 7U, "ADC PWM PAGE");
}

static void RenderPage_ADC(void)
{
  OLED_DrawString6x8(0U, 0U, "ADC TEST");
  DrawStatusLine(1U, "P1RAW", (int32_t)adc_buf[0]);
  DrawStatusLine(2U, "P2RAW", (int32_t)adc_buf[1]);
  DrawStatusLine(3U, "P3RAW", (int32_t)adc_buf[2]);
  DrawStatusLine(4U, "FWD", (int32_t)GetDrivePwmPermille());
  DrawStatusLine(5U, "TURN", (int32_t)GetTurnPwmPermille());
  DrawStatusLine(6U, "SAFE", (int32_t)GetObstacleSafeDistanceMm());
  OLED_DrawString6x8(0U, 7U, "P1FWD P2TURN P3SAFE");
}

static void RenderPage_Encoder(void)
{
  MotorControlState_t motor_state = {0};
  const char *mode_text = "MOTOR OFF";

  (void)MotorControl_GetState(&motor_state);

  if (motor_state.mode == 1U)
  {
    mode_text = "MOTOR FWD";
  }
  else if (motor_state.mode == 2U)
  {
    mode_text = "MOTOR LEFT";
  }
  else if (motor_state.mode == 3U)
  {
    mode_text = "MOTOR RIGHT";
  }
  else if (motor_state.mode == 4U)
  {
    mode_text = "MOTOR BRAKE";
  }

  OLED_DrawString6x8(0U, 0U, mode_text);
  DrawStatusLine(1U, "LDELTA", (int32_t)motor_state.left_delta);
  DrawStatusLine(2U, "RDELTA", (int32_t)motor_state.right_delta);
  DrawStatusLine(3U, "LDUTY", (int32_t)motor_state.left_duty_permille);
  DrawStatusLine(4U, "RDUTY", (int32_t)motor_state.right_duty_permille);
  DrawStatusLine(5U, "ERR", motor_state.balance_error);
  DrawStatusLine(6U, "CORR", motor_state.correction_permille);
  OLED_DrawString6x8(0U, 7U, "BT 0STOP 1FWD 2L 3R");
}

static void TestApp_UpdateDisplay(void)
{
  uint32_t now = HAL_GetTick();

  if (!oled_ready)
  {
    if ((now - last_oled_recover_tick_ms) >= OLED_RECOVER_RETRY_MS)
    {
      last_oled_recover_tick_ms = now;
      (void)HAL_I2C_DeInit(&hi2c2);
      MX_I2C2_Init();
      oled_ready = OLED_InitMinimal();
    }
    return;
  }

  if ((now - last_oled_tick_ms) < 150U)
  {
    return;
  }

  last_oled_tick_ms = now;
  OLED_Clear();

  switch (current_page)
  {
    case TEST_PAGE_LIDAR:
      RenderPage_LiDAR();
      break;
    case TEST_PAGE_MPU:
      RenderPage_MPU();
      break;
    case TEST_PAGE_ADC:
      RenderPage_ADC();
      break;
    case TEST_PAGE_ENCODER:
    default:
      RenderPage_Encoder();
      break;
  }

  OLED_Update();
}

static void TestApp_UpdateSensors(void)
{
  uint16_t left_now;
  uint16_t right_now;
  uint32_t now = HAL_GetTick();

  if ((now - last_sensor_tick_ms) < 20U)
  {
    return;
  }

  last_sensor_tick_ms = now;

  if (LidarPipeline_GetLatestResult(&lidar_result))
  {
    lidar_result_valid = true;
    last_lidar_result_tick_ms = now;
  }

  Mpu6500_Update();
  (void)Mpu6500_GetState(&mpu_state);

  left_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
  right_now = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);

  encoder_test.left_delta = Encoder_ComputeDelta(left_now, encoder_test.last_left_counter);
  encoder_test.right_delta =
      (int16_t)((int32_t)Encoder_ComputeDelta(right_now, encoder_test.last_right_counter) * ENCODER_RIGHT_DELTA_SIGN);
  encoder_test.left_total += encoder_test.left_delta;
  encoder_test.right_total += encoder_test.right_delta;
  encoder_test.last_left_counter = left_now;
  encoder_test.last_right_counter = right_now;

  TestApp_UpdateGyroDriftCompensation(encoder_test.left_delta, encoder_test.right_delta);
  TestApp_UpdateOdomDebug(now, encoder_test.left_delta, encoder_test.right_delta);
  TestApp_UpdateMappingPose(now, encoder_test.left_delta, encoder_test.right_delta);
  TestApp_UpdateAngleTurn(now);

  TestApp_ProcessLidarPoints();
  TestApp_UpdateNavigation(now);
  TestApp_UpdateMotorSpeedFromAdc();
}

static void TestApp_InitPeripherals(void)
{
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  (void)HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buf, ADC_CHANNEL_COUNT);
  (void)HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  if (!MotorControl_Init())
  {
    Error_Handler();
  }

  encoder_test.last_left_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim2) & 0xFFFFU);
  encoder_test.last_right_counter = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim4) & 0xFFFFU);

  oled_ready = OLED_InitMinimal();
  (void)Mpu6500_Init();
  (void)Mpu6500_GetState(&mpu_state);
  (void)BluetoothControl_Init();
  MappingGrid_Init();
  if (!LidarPipeline_Init())
  {
    Error_Handler();
  }
}

static void TestApp_HandleBluetoothCommands(void)
{
  BluetoothCommand_t command;

  while (BluetoothControl_TakeCommand(&command))
  {
    switch (command.type)
    {
      case BLUETOOTH_CMD_DRIVE_FORWARD:
        TestApp_StopNavigation("MANUAL");
        TestApp_StopAngleTurn(false);
        MotorControl_SetForward(GetDrivePwmPermille());
        (void)BluetoothControl_SendText("MOTOR forward\r\n");
        break;

      case BLUETOOTH_CMD_TURN_LEFT:
        TestApp_StopNavigation("MANUAL");
        TestApp_StopAngleTurn(false);
        MotorControl_SetTurnLeft(GetTurnPwmPermille());
        (void)BluetoothControl_SendText("MOTOR left\r\n");
        break;

      case BLUETOOTH_CMD_TURN_RIGHT:
        TestApp_StopNavigation("MANUAL");
        TestApp_StopAngleTurn(false);
        MotorControl_SetTurnRight(GetTurnPwmPermille());
        (void)BluetoothControl_SendText("MOTOR right\r\n");
        break;

      case BLUETOOTH_CMD_DRIVE_STOP:
        TestApp_StopNavigation("ESTOP");
        TestApp_StopAngleTurn(false);
        MotorControl_Stop();
        (void)BluetoothControl_SendText("MOTOR estop\r\n");
        break;

      case BLUETOOTH_CMD_STOP_ALL:
        TestApp_StopNavigation("MAP STOP");
        TestApp_StopAngleTurn(false);
        MotorControl_Stop();
        TestApp_StopMapping();
        lidar_debug_active = false;
        TestApp_StopOdomDebug();
        (void)BluetoothControl_SendText("MAP STOP\r\n");
        break;

      case BLUETOOTH_CMD_START_MAPPING:
        TestApp_StopNavigation("MAP");
        TestApp_StopAngleTurn(false);
        MotorControl_Stop();
        TestApp_StartMapping();
        break;

      case BLUETOOTH_CMD_LIDAR_DEBUG_ON:
        lidar_debug_active = true;
        lidar_debug_seq = 0U;
        (void)BluetoothControl_SendText("LIDAR START format=LP seq=<n> a=<cdeg> d=<mm> q=<quality> s=<scan_start>\r\n");
        break;

      case BLUETOOTH_CMD_LIDAR_DEBUG_OFF:
        lidar_debug_active = false;
        (void)BluetoothControl_SendText("LIDAR STOP\r\n");
        break;

      case BLUETOOTH_CMD_ODOM_DEBUG_ON:
        TestApp_StartOdomDebug();
        break;

      case BLUETOOTH_CMD_ODOM_DEBUG_OFF:
        TestApp_StopOdomDebug();
        break;

      case BLUETOOTH_CMD_NAV_SET_START:
        if (TestApp_ParseMazeCell(command.text, &nav_start_cell))
        {
          char line[64];
          if (!mapping_active)
          {
            TestApp_SetPoseToMazeCell(&nav_start_cell);
          }
          (void)snprintf(
              line,
              sizeof(line),
              "NAV START CELL %u,%u pose=%ld,%ld %s\r\n",
              (unsigned int)nav_start_cell.x,
              (unsigned int)nav_start_cell.y,
              (long)mapping_pose.x_mm,
              (long)mapping_pose.y_mm,
              mapping_active ? "pose-kept" : "pose-set");
          (void)BluetoothControl_SendText(line);
        }
        else
        {
          (void)BluetoothControl_SendText("NAV ERR start use Sxy, example S00\r\n");
        }
        break;

      case BLUETOOTH_CMD_NAV_SET_GOAL:
        if (TestApp_ParseMazeCell(command.text, &nav_goal_cell))
        {
          char line[40];
          (void)snprintf(
              line,
              sizeof(line),
              "NAV GOAL CELL %u,%u\r\n",
              (unsigned int)nav_goal_cell.x,
              (unsigned int)nav_goal_cell.y);
          (void)BluetoothControl_SendText(line);
        }
        else
        {
          (void)BluetoothControl_SendText("NAV ERR goal use Gxy, example G44\r\n");
        }
        break;

      case BLUETOOTH_CMD_NAV_RUN:
        TestApp_StartNavigation();
        break;

      case BLUETOOTH_CMD_NAV_STOP:
        TestApp_StopNavigation("CMD");
        break;

      case BLUETOOTH_CMD_TURN_LEFT_DEG:
        TestApp_StopNavigation("MANUAL");
        TestApp_StartAngleTurn(-1, TestApp_ParseTurnDegrees(command.text));
        break;

      case BLUETOOTH_CMD_TURN_RIGHT_DEG:
        TestApp_StopNavigation("MANUAL");
        TestApp_StartAngleTurn(1, TestApp_ParseTurnDegrees(command.text));
        break;

      case BLUETOOTH_CMD_SHOW_MAP_RESULT:
        TestApp_RequestFullMapStream();
        (void)BluetoothControl_SendText("MAP SHOW\r\n");
        break;

      case BLUETOOTH_CMD_CLEAR_MAP:
        TestApp_StopNavigation("CLEAR");
        TestApp_StopAngleTurn(false);
        MotorControl_Stop();
        TestApp_ClearMappingData();
        break;

      case BLUETOOTH_CMD_NONE:
      case BLUETOOTH_CMD_DEBUG_ON:
      case BLUETOOTH_CMD_DEBUG_OFF:
      case BLUETOOTH_CMD_UNKNOWN:
      default:
        break;
    }
  }
}

static void TestApp_StartMapping(void)
{
  mapping_pose.x_mm = MAPPING_START_X_MM;
  mapping_pose.y_mm = MAPPING_START_Y_MM;
  mapping_pose.heading_cdeg = MAPPING_START_HEADING_CDEG;
  mapping_travel_residual_x1000 = 0L;
  MappingGrid_Reset();
  MappingGrid_SetPose(&mapping_pose);
  TestApp_ResetPoseHistory();
  TestApp_ResetMappingScanState();
  TestApp_ResetMappingDiagnostics();

  mapping_active = true;
  last_mapping_pose_tick_ms = HAL_GetTick();
  mapping_start_tick_ms = last_mapping_pose_tick_ms;
  TestApp_RecordPoseHistory(last_mapping_pose_tick_ms);
  last_map_row_tx_tick_ms = 0U;
  last_map_stat_tx_tick_ms = 0U;
  last_pose_tx_tick_ms = 0U;
  next_map_tx_row = 0U;

  TestApp_SendMapHeader("START");
}

static void TestApp_StopMapping(void)
{
  if (!mapping_active)
  {
    return;
  }

  mapping_active = false;
  TestApp_ResetMappingScanState();
  TestApp_SendMapHeader("STOP");
  TestApp_SendMapStat();
  TestApp_SendMapDiag();
}

static void TestApp_ClearMappingData(void)
{
  uint32_t now = HAL_GetTick();

  MappingGrid_Reset();
  MappingGrid_SetPose(&mapping_pose);
  TestApp_ResetPoseHistory();
  TestApp_RecordPoseHistory(now);
  TestApp_ResetMappingScanState();
  TestApp_ResetMappingDiagnostics();

  mapping_start_tick_ms = now;
  last_mapping_pose_tick_ms = now;
  last_map_row_tx_tick_ms = 0U;
  last_map_stat_tx_tick_ms = 0U;
  last_pose_tx_tick_ms = 0U;
  next_map_tx_row = 0U;

  TestApp_SendMapHeader("CLEAR");
  TestApp_SendMapStat();
  TestApp_SendMapDiag();
}

static void TestApp_ProcessLidarPoints(void)
{
  LidarPoint_t point;
  uint32_t now_ms = HAL_GetTick();
  uint8_t count = 0U;
  uint8_t debug_tx_count = 0U;

  while ((count < MAPPING_POINT_BATCH_LIMIT) && LidarPipeline_TakePoint(&point))
  {
    bool point_is_fresh = ((int32_t)(point.tick_count - mapping_start_tick_ms) >= 0L) &&
        ((now_ms - point.tick_count) <= MAPPING_LIDAR_POINT_MAX_AGE_MS);

    TestApp_HandleMappingScanPoint(&point, point_is_fresh);

    if (point_is_fresh)
    {
      TestApp_UpdateNavObstacle(&point, now_ms);
    }

    if (lidar_debug_active &&
        ((debug_tx_count < LIDAR_DEBUG_MAX_TX_PER_BATCH) ||
         ((point.flags & LIDAR_POINT_FLAG_SCAN_START) != 0U)))
    {
      TestApp_SendLidarDebugPoint(&point);
      debug_tx_count++;
    }

    count++;
  }
}

static void TestApp_SendLidarDebugPoint(const LidarPoint_t *point)
{
  char line[80];

  if (point == NULL)
  {
    return;
  }

  (void)snprintf(
      line,
      sizeof(line),
      "LP seq=%lu a=%u d=%u q=%u s=%u\r\n",
      (unsigned long)lidar_debug_seq++,
      (unsigned int)point->angle_cdeg,
      (unsigned int)point->distance_mm,
      (unsigned int)point->quality,
      (unsigned int)((point->flags & LIDAR_POINT_FLAG_SCAN_START) != 0U));
  (void)BluetoothControl_SendText(line);
}

static void TestApp_ResetMappingScanState(void)
{
  memset(mapping_scan_points, 0, sizeof(mapping_scan_points));
  mapping_scan_start_tick_ms = 0U;
  mapping_scan_start_angle_cdeg = 0U;
  mapping_scan_count = 0U;
  mapping_scan_open = false;
  mapping_scan_motion_blocked = false;
}

static void TestApp_ResetMappingDiagnostics(void)
{
  memset(&mapping_diag, 0, sizeof(mapping_diag));
}

static void TestApp_HandleMappingScanPoint(const LidarPoint_t *point, bool point_is_fresh)
{
  if ((point == NULL) || !mapping_active || !point_is_fresh)
  {
    return;
  }

  if ((point->flags & LIDAR_POINT_FLAG_SCAN_START) != 0U)
  {
    if (mapping_scan_open)
    {
      TestApp_ProcessCompletedMappingScan(point->tick_count);
    }
    TestApp_StartMappingScan(point);
  }
  else if (!mapping_scan_open)
  {
    TestApp_StartMappingScan(point);
  }

  if ((point->distance_mm == 0U) || (point->quality < MAPPING_POINT_MIN_QUALITY))
  {
    mapping_diag.points_quality_rejected++;
    return;
  }

  if (TestApp_ShouldHoldMappingUpdates())
  {
    mapping_scan_motion_blocked = true;
  }

  TestApp_BufferMappingScanPoint(point);
}

static void TestApp_StartMappingScan(const LidarPoint_t *point)
{
  mapping_scan_count = 0U;
  mapping_scan_motion_blocked = TestApp_ShouldHoldMappingUpdates();
  mapping_scan_open = true;

  if (point != NULL)
  {
    mapping_scan_start_tick_ms = point->tick_count;
    mapping_scan_start_angle_cdeg = point->angle_cdeg;
  }
  else
  {
    mapping_scan_start_tick_ms = HAL_GetTick();
    mapping_scan_start_angle_cdeg = 0U;
  }

  mapping_diag.scans_started++;
}

static void TestApp_BufferMappingScanPoint(const LidarPoint_t *point)
{
  mapping_scan_point_t *scan_point;

  if ((point == NULL) || !mapping_scan_open)
  {
    return;
  }

  if (mapping_scan_count >= MAPPING_SCAN_BUFFER_POINTS)
  {
    mapping_diag.points_overflowed++;
    return;
  }

  scan_point = &mapping_scan_points[mapping_scan_count++];
  memset(scan_point, 0, sizeof(*scan_point));
  scan_point->tick_ms = point->tick_count;
  scan_point->angle_cdeg = point->angle_cdeg;
  scan_point->distance_mm = point->distance_mm;
  scan_point->quality = point->quality;
  mapping_diag.points_buffered++;
}

static void TestApp_ProcessCompletedMappingScan(uint32_t end_tick_ms)
{
  MappingGridStats_t stats;
  MappingGridPose_t base_end_pose;
  MappingGridPose_t matched_end_pose;
  int32_t match_score = 0L;
  uint16_t match_used = 0U;
  uint16_t valid_points = 0U;
  uint16_t inserted_points;
  bool matched = false;

  mapping_diag.scans_completed++;
  mapping_diag.last_scan_points = mapping_scan_count;
  mapping_diag.last_scan_inserted = 0U;
  mapping_diag.last_match_used = 0U;
  mapping_diag.last_match_score = 0L;
  mapping_diag.last_match_dx_mm = 0L;
  mapping_diag.last_match_dy_mm = 0L;
  mapping_diag.last_match_dtheta_cdeg = 0L;

  if (mapping_scan_count < MAPPING_SCAN_MIN_POINTS)
  {
    mapping_diag.scans_short++;
    return;
  }

  if (mapping_scan_motion_blocked)
  {
    mapping_diag.scans_motion_held++;
    return;
  }

  if (!TestApp_PrepareMappingScan(end_tick_ms, &valid_points) ||
      (valid_points < MAPPING_SCAN_MIN_POINTS))
  {
    mapping_diag.pose_misses++;
    return;
  }

  if (!TestApp_GetPoseForTick(end_tick_ms, &base_end_pose))
  {
    base_end_pose = mapping_pose;
  }

  matched_end_pose = base_end_pose;
  if (MappingGrid_GetStats(&stats) &&
      (stats.occupied_cells >= MAPPING_MATCH_MIN_OCCUPIED_CELLS))
  {
    matched = TestApp_MatchMappingScan(&base_end_pose, &matched_end_pose, &match_score, &match_used);
    mapping_diag.last_match_score = match_score;
    mapping_diag.last_match_used = match_used;
  }

  if (matched)
  {
    mapping_diag.scans_matched++;
    mapping_diag.last_match_dx_mm = matched_end_pose.x_mm - base_end_pose.x_mm;
    mapping_diag.last_match_dy_mm = matched_end_pose.y_mm - base_end_pose.y_mm;
    mapping_diag.last_match_dtheta_cdeg =
        TestApp_SignedHeadingErrorCdeg(matched_end_pose.heading_cdeg, base_end_pose.heading_cdeg);
  }
  else
  {
    mapping_diag.scans_unmatched++;
  }

  inserted_points = TestApp_InsertPreparedMappingScan(&base_end_pose, &matched_end_pose);
  mapping_diag.last_scan_inserted = inserted_points;
  mapping_diag.points_inserted += inserted_points;
  mapping_diag.scans_inserted++;
}

static bool TestApp_PrepareMappingScan(uint32_t end_tick_ms, uint16_t *out_valid_points)
{
  uint32_t scan_period_ms = end_tick_ms - mapping_scan_start_tick_ms;
  uint16_t i;
  uint16_t valid_points = 0U;

  if ((scan_period_ms < MAPPING_SCAN_MIN_PERIOD_MS) ||
      (scan_period_ms > MAPPING_SCAN_MAX_PERIOD_MS))
  {
    scan_period_ms = MAPPING_SCAN_DEFAULT_PERIOD_MS;
  }

  for (i = 0U; i < mapping_scan_count; ++i)
  {
    uint32_t tick_ms = TestApp_EstimateScanPointTick(mapping_scan_points[i].angle_cdeg, scan_period_ms);

    if ((int32_t)(tick_ms - end_tick_ms) > 0L)
    {
      tick_ms = end_tick_ms;
    }

    mapping_scan_points[i].tick_ms = tick_ms;
    if (TestApp_GetPoseForTick(tick_ms, &mapping_scan_points[i].pose))
    {
      valid_points++;
    }
    else
    {
      mapping_scan_points[i].quality = 0U;
    }
  }

  valid_points = TestApp_FilterMappingScanOutliers();

  if (out_valid_points != NULL)
  {
    *out_valid_points = valid_points;
  }

  return (valid_points > 0U);
}

static bool TestApp_ShouldHoldMappingUpdates(void)
{
  MotorControlState_t motor_state = {0};

  if (angle_turn_active)
  {
    return true;
  }

  if (AppAbs32(gyro_z_corrected_dps_x100) > MAPPING_MOTION_GYRO_HOLD_DPS_X100)
  {
    return true;
  }

  if (MotorControl_GetState(&motor_state) &&
      !motor_state.forward_active &&
      (motor_state.duty_permille > 0U))
  {
    return true;
  }

  return false;
}

static uint16_t TestApp_AngleDeltaAbsCdeg(uint16_t a_cdeg, uint16_t b_cdeg)
{
  int32_t diff = (int32_t)a_cdeg - (int32_t)b_cdeg;

  if (diff < 0L)
  {
    diff = -diff;
  }
  if (diff > 18000L)
  {
    diff = 36000L - diff;
  }
  return (uint16_t)diff;
}

static bool TestApp_AreScanPointsContinuous(const mapping_scan_point_t *a, const mapping_scan_point_t *b)
{
  int32_t angle_delta_cdeg;
  int32_t range_delta_mm;
  float angle_delta_rad;
  float endpoint_gap_sq;
  float max_gap_sq;

  if ((a == NULL) ||
      (b == NULL) ||
      (a->quality < MAPPING_POINT_MIN_QUALITY) ||
      (b->quality < MAPPING_POINT_MIN_QUALITY))
  {
    return false;
  }

  angle_delta_cdeg = (int32_t)TestApp_AngleDeltaAbsCdeg(a->angle_cdeg, b->angle_cdeg);
  if (angle_delta_cdeg > (int32_t)MAPPING_SCAN_CLUSTER_ANGLE_CDEG)
  {
    return false;
  }

  range_delta_mm = (int32_t)a->distance_mm - (int32_t)b->distance_mm;
  if (range_delta_mm < 0L)
  {
    range_delta_mm = -range_delta_mm;
  }
  if (range_delta_mm <= (int32_t)MAPPING_SCAN_CLUSTER_RANGE_MM)
  {
    return true;
  }

  angle_delta_rad = ((float)angle_delta_cdeg * MAPPING_PI) / 18000.0f;
  endpoint_gap_sq =
      ((float)a->distance_mm * (float)a->distance_mm) +
      ((float)b->distance_mm * (float)b->distance_mm) -
      (2.0f * (float)a->distance_mm * (float)b->distance_mm * cosf(angle_delta_rad));
  max_gap_sq = (float)MAPPING_SCAN_CLUSTER_RANGE_MM * (float)MAPPING_SCAN_CLUSTER_RANGE_MM;
  return endpoint_gap_sq <= max_gap_sq;
}

static uint16_t TestApp_FilterMappingScanOutliers(void)
{
  bool keep[MAPPING_SCAN_BUFFER_POINTS];
  uint16_t i;
  uint16_t kept = 0U;

  memset(keep, 0, sizeof(keep));

  for (i = 0U; i < mapping_scan_count; ++i)
  {
    uint16_t neighbors = 0U;
    uint16_t offset;

    if (mapping_scan_points[i].quality < MAPPING_POINT_MIN_QUALITY)
    {
      continue;
    }

    for (offset = 1U; offset <= MAPPING_SCAN_CLUSTER_LOOKAHEAD; ++offset)
    {
      if ((i >= offset) &&
          TestApp_AreScanPointsContinuous(&mapping_scan_points[i], &mapping_scan_points[i - offset]))
      {
        neighbors++;
      }
      if (((uint16_t)(i + offset) < mapping_scan_count) &&
          TestApp_AreScanPointsContinuous(&mapping_scan_points[i], &mapping_scan_points[i + offset]))
      {
        neighbors++;
      }
    }

    if (neighbors >= (uint16_t)(MAPPING_SCAN_MIN_CLUSTER_POINTS - 1U))
    {
      keep[i] = true;
      kept++;
    }
  }

  for (i = 0U; i < mapping_scan_count; ++i)
  {
    if (!keep[i])
    {
      mapping_scan_points[i].quality = 0U;
      mapping_diag.points_quality_rejected++;
    }
  }

  return kept;
}

static bool TestApp_MatchMappingScan(const MappingGridPose_t *base_end_pose,
                                     MappingGridPose_t *out_matched_end_pose,
                                     int32_t *out_score,
                                     uint16_t *out_used_points)
{
  MappingGridPose_t candidate_pose;
  MappingGridPose_t best_pose;
  int32_t best_score = -2147483647L;
  int32_t dtheta;
  int32_t dy;
  int32_t dx;
  int32_t best_cost = 2147483647L;
  uint16_t best_used = 0U;
  bool found = false;

  if ((base_end_pose == NULL) || (out_matched_end_pose == NULL))
  {
    return false;
  }

  best_pose = *base_end_pose;

  for (dtheta = -MAPPING_MATCH_HEADING_RANGE_CDEG;
       dtheta <= MAPPING_MATCH_HEADING_RANGE_CDEG;
       dtheta += MAPPING_MATCH_HEADING_STEP_CDEG)
  {
    for (dy = -MAPPING_MATCH_XY_RANGE_MM;
         dy <= MAPPING_MATCH_XY_RANGE_MM;
         dy += MAPPING_MATCH_XY_STEP_MM)
    {
      for (dx = -MAPPING_MATCH_XY_RANGE_MM;
           dx <= MAPPING_MATCH_XY_RANGE_MM;
           dx += MAPPING_MATCH_XY_STEP_MM)
      {
        uint16_t used_points = 0U;
        int32_t score;
        int32_t candidate_cost;

        candidate_pose.x_mm = base_end_pose->x_mm + dx;
        candidate_pose.y_mm = base_end_pose->y_mm + dy;
        candidate_pose.heading_cdeg = NormalizeHeadingCdeg(base_end_pose->heading_cdeg + dtheta);

        score = TestApp_ScoreMappingScanCandidate(base_end_pose, &candidate_pose, &used_points);
        candidate_cost = AppAbs32(dx) + AppAbs32(dy) + (AppAbs32(dtheta) / 10L);
        if ((used_points >= MAPPING_MATCH_MIN_USED_POINTS) &&
            ((score > best_score) ||
             ((score == best_score) && (candidate_cost < best_cost))))
        {
          best_score = score;
          best_pose = candidate_pose;
          best_cost = candidate_cost;
          best_used = used_points;
          found = true;
        }
      }
    }
  }

  if (out_score != NULL)
  {
    *out_score = found ? best_score : 0L;
  }
  if (out_used_points != NULL)
  {
    *out_used_points = found ? best_used : 0U;
  }

  if (!found || (best_score < MAPPING_MATCH_MIN_SCORE))
  {
    return false;
  }

  *out_matched_end_pose = best_pose;
  return true;
}

static int32_t TestApp_ScoreMappingScanCandidate(const MappingGridPose_t *base_end_pose,
                                                 const MappingGridPose_t *candidate_end_pose,
                                                 uint16_t *out_used_points)
{
  uint16_t i;
  uint16_t used_points = 0U;
  int32_t score = 0L;

  if ((base_end_pose == NULL) || (candidate_end_pose == NULL))
  {
    if (out_used_points != NULL)
    {
      *out_used_points = 0U;
    }
    return -2147483647L;
  }

  for (i = 0U; i < mapping_scan_count; i = (uint16_t)(i + MAPPING_SCAN_SCORE_STRIDE))
  {
    MappingGridPose_t corrected_pose;
    int16_t point_score;

    if (mapping_scan_points[i].quality < MAPPING_POINT_MIN_QUALITY)
    {
      continue;
    }

    corrected_pose = TestApp_ApplyScanPoseCorrection(&mapping_scan_points[i].pose,
                                                     base_end_pose,
                                                     candidate_end_pose);
    point_score = MappingGrid_ScorePolarPointAtPose(&corrected_pose,
                                                    mapping_scan_points[i].angle_cdeg,
                                                    mapping_scan_points[i].distance_mm,
                                                    mapping_scan_points[i].quality);
    score += point_score;
    if (point_score > -2)
    {
      used_points++;
    }
  }

  if (out_used_points != NULL)
  {
    *out_used_points = used_points;
  }

  return score;
}

static MappingGridPose_t TestApp_ApplyScanPoseCorrection(const MappingGridPose_t *point_pose,
                                                         const MappingGridPose_t *base_end_pose,
                                                         const MappingGridPose_t *candidate_end_pose)
{
  MappingGridPose_t corrected_pose = {0};
  int32_t heading_delta_cdeg;
  float heading_delta_rad;
  float cos_delta;
  float sin_delta;
  float rel_x;
  float rel_y;

  if ((point_pose == NULL) || (base_end_pose == NULL) || (candidate_end_pose == NULL))
  {
    return corrected_pose;
  }

  heading_delta_cdeg =
      TestApp_SignedHeadingErrorCdeg(candidate_end_pose->heading_cdeg, base_end_pose->heading_cdeg);
  heading_delta_rad = ((float)heading_delta_cdeg * MAPPING_PI) / 18000.0f;
  cos_delta = cosf(heading_delta_rad);
  sin_delta = sinf(heading_delta_rad);
  rel_x = (float)(point_pose->x_mm - base_end_pose->x_mm);
  rel_y = (float)(point_pose->y_mm - base_end_pose->y_mm);

  corrected_pose.x_mm = candidate_end_pose->x_mm + (int32_t)((rel_x * cos_delta) - (rel_y * sin_delta));
  corrected_pose.y_mm = candidate_end_pose->y_mm + (int32_t)((rel_x * sin_delta) + (rel_y * cos_delta));
  corrected_pose.heading_cdeg = NormalizeHeadingCdeg(point_pose->heading_cdeg + heading_delta_cdeg);

  return corrected_pose;
}

static uint16_t TestApp_InsertPreparedMappingScan(const MappingGridPose_t *base_end_pose,
                                                  const MappingGridPose_t *matched_end_pose)
{
  uint16_t i;
  uint16_t inserted_points = 0U;

  if ((base_end_pose == NULL) || (matched_end_pose == NULL))
  {
    return 0U;
  }

  for (i = 0U; i < mapping_scan_count; ++i)
  {
    MappingGridPose_t corrected_pose;

    if (mapping_scan_points[i].quality < MAPPING_POINT_MIN_QUALITY)
    {
      continue;
    }

    corrected_pose = TestApp_ApplyScanPoseCorrection(&mapping_scan_points[i].pose,
                                                     base_end_pose,
                                                     matched_end_pose);
    if (MappingGrid_InsertPolarPointAtPose(&corrected_pose,
                                           mapping_scan_points[i].angle_cdeg,
                                           mapping_scan_points[i].distance_mm,
                                           mapping_scan_points[i].quality))
    {
      inserted_points++;
    }
  }

  return inserted_points;
}

static uint32_t TestApp_EstimateScanPointTick(uint16_t angle_cdeg, uint32_t scan_period_ms)
{
  int32_t relative_angle_cdeg =
      NormalizeHeadingCdeg((int32_t)angle_cdeg - (int32_t)mapping_scan_start_angle_cdeg);

  return mapping_scan_start_tick_ms +
      (uint32_t)(((uint64_t)scan_period_ms * (uint32_t)relative_angle_cdeg) / 36000ULL);
}

static void TestApp_UpdateMappingPose(uint32_t now_ms, int16_t left_delta, int16_t right_delta)
{
  MotorControlState_t motor_state = {0};
  uint32_t delta_ms;
  int32_t gyro_delta_cdeg = 0L;
  int32_t heading_for_travel_cdeg;
  int32_t travel_counts;
  int32_t travel_x1000;
  int32_t travel_mm;
  float heading_rad;

  if (!mapping_active)
  {
    last_mapping_pose_tick_ms = now_ms;
    return;
  }

  if (last_mapping_pose_tick_ms == 0U)
  {
    last_mapping_pose_tick_ms = now_ms;
    MappingGrid_SetPose(&mapping_pose);
    return;
  }

  delta_ms = now_ms - last_mapping_pose_tick_ms;
  last_mapping_pose_tick_ms = now_ms;

  if (mpu_state.ready)
  {
    gyro_delta_cdeg = (gyro_z_corrected_dps_x100 * (int32_t)delta_ms) / 1000L;
    mapping_pose.heading_cdeg = NormalizeHeadingCdeg(mapping_pose.heading_cdeg + gyro_delta_cdeg);
  }

  if (MotorControl_GetState(&motor_state) && motor_state.forward_active)
  {
    travel_counts = (AppAbs32((int32_t)left_delta) + AppAbs32((int32_t)right_delta)) / 2L;
    travel_x1000 = (travel_counts * MAPPING_ENCODER_MM_PER_COUNT_X1000) + mapping_travel_residual_x1000;
    travel_mm = travel_x1000 / 1000L;
    mapping_travel_residual_x1000 = travel_x1000 - (travel_mm * 1000L);
    if (travel_mm != 0L)
    {
      heading_for_travel_cdeg = NormalizeHeadingCdeg(mapping_pose.heading_cdeg - (gyro_delta_cdeg / 2L));
      heading_rad = ((float)heading_for_travel_cdeg * MAPPING_PI) / 18000.0f;
      mapping_pose.x_mm += (int32_t)((float)travel_mm * cosf(heading_rad));
      mapping_pose.y_mm += (int32_t)((float)travel_mm * sinf(heading_rad));
    }
  }

  MappingGrid_SetPose(&mapping_pose);
  TestApp_RecordPoseHistory(now_ms);
}

static void TestApp_ResetPoseHistory(void)
{
  memset(mapping_pose_history, 0, sizeof(mapping_pose_history));
  memset(mapping_pose_history_ticks, 0, sizeof(mapping_pose_history_ticks));
  mapping_pose_history_next = 0U;
  mapping_pose_history_count = 0U;
}

static void TestApp_RecordPoseHistory(uint32_t tick_ms)
{
  mapping_pose_history[mapping_pose_history_next] = mapping_pose;
  mapping_pose_history_ticks[mapping_pose_history_next] = tick_ms;
  mapping_pose_history_next++;
  if (mapping_pose_history_next >= MAPPING_POSE_HISTORY_LENGTH)
  {
    mapping_pose_history_next = 0U;
  }

  if (mapping_pose_history_count < MAPPING_POSE_HISTORY_LENGTH)
  {
    mapping_pose_history_count++;
  }
}

static bool TestApp_GetPoseForTick(uint32_t tick_ms, MappingGridPose_t *out_pose)
{
  uint8_t i;
  uint8_t before_index = 0U;
  uint8_t after_index = 0U;
  bool have_before = false;
  bool have_after = false;

  if ((out_pose == NULL) || (mapping_pose_history_count == 0U))
  {
    return false;
  }

  for (i = 0U; i < mapping_pose_history_count; ++i)
  {
    uint32_t pose_tick = mapping_pose_history_ticks[i];
    int32_t diff = (int32_t)(pose_tick - tick_ms);

    if (diff == 0L)
    {
      *out_pose = mapping_pose_history[i];
      return true;
    }

    if (diff < 0L)
    {
      if (!have_before ||
          ((int32_t)(pose_tick - mapping_pose_history_ticks[before_index]) > 0L))
      {
        before_index = i;
        have_before = true;
      }
    }
    else
    {
      if (!have_after ||
          ((int32_t)(pose_tick - mapping_pose_history_ticks[after_index]) < 0L))
      {
        after_index = i;
        have_after = true;
      }
    }
  }

  if (have_before && have_after)
  {
    uint32_t before_tick = mapping_pose_history_ticks[before_index];
    uint32_t after_tick = mapping_pose_history_ticks[after_index];
    uint32_t span = after_tick - before_tick;
    uint32_t offset = tick_ms - before_tick;
    MappingGridPose_t before_pose = mapping_pose_history[before_index];
    MappingGridPose_t after_pose = mapping_pose_history[after_index];
    int32_t heading_delta =
        TestApp_SignedHeadingErrorCdeg(after_pose.heading_cdeg, before_pose.heading_cdeg);

    if (span == 0U)
    {
      *out_pose = before_pose;
      return true;
    }

    out_pose->x_mm = before_pose.x_mm +
        (int32_t)(((int64_t)(after_pose.x_mm - before_pose.x_mm) * (int64_t)offset) / (int64_t)span);
    out_pose->y_mm = before_pose.y_mm +
        (int32_t)(((int64_t)(after_pose.y_mm - before_pose.y_mm) * (int64_t)offset) / (int64_t)span);
    out_pose->heading_cdeg = NormalizeHeadingCdeg(before_pose.heading_cdeg +
        (int32_t)(((int64_t)heading_delta * (int64_t)offset) / (int64_t)span));
    return true;
  }

  if (have_before)
  {
    *out_pose = mapping_pose_history[before_index];
    return true;
  }

  if (have_after)
  {
    *out_pose = mapping_pose_history[after_index];
    return true;
  }

  return false;
}

static bool TestApp_ParseMazeCell(const char *text, maze_cell_t *out_cell)
{
  uint8_t digits[2] = {0U, 0U};
  uint8_t count = 0U;

  if ((text == NULL) || (out_cell == NULL))
  {
    return false;
  }

  while ((*text != '\0') && (count < 2U))
  {
    if ((*text >= '0') && (*text <= '4'))
    {
      digits[count++] = (uint8_t)(*text - '0');
    }
    text++;
  }

  if (count < 2U)
  {
    return false;
  }

  out_cell->x = digits[0];
  out_cell->y = digits[1];
  return TestApp_IsMazeCellValid(out_cell);
}

static bool TestApp_IsMazeCellValid(const maze_cell_t *cell)
{
  return ((cell != NULL) && (cell->x < MAZE_GRID_CELLS) && (cell->y < MAZE_GRID_CELLS));
}

static int32_t TestApp_MazeCellCenterX(uint8_t x)
{
  return MAZE_ORIGIN_X_MM + ((int32_t)x * MAZE_CELL_SIZE_MM) + (MAZE_CELL_SIZE_MM / 2L);
}

static int32_t TestApp_MazeCellCenterY(uint8_t y)
{
  return MAZE_ORIGIN_Y_MM + ((int32_t)y * MAZE_CELL_SIZE_MM) + (MAZE_CELL_SIZE_MM / 2L);
}

static uint16_t TestApp_MazeCellManhattan(const maze_cell_t *a, const maze_cell_t *b)
{
  if ((a == NULL) || (b == NULL))
  {
    return UINT16_MAX;
  }

  return (uint16_t)(AppAbs32((int32_t)a->x - (int32_t)b->x) +
                    AppAbs32((int32_t)a->y - (int32_t)b->y));
}

static void TestApp_SetPoseToMazeCell(const maze_cell_t *cell)
{
  if (!TestApp_IsMazeCellValid(cell))
  {
    return;
  }

  mapping_pose.x_mm = TestApp_MazeCellCenterX(cell->x);
  mapping_pose.y_mm = TestApp_MazeCellCenterY(cell->y);
  mapping_travel_residual_x1000 = 0L;
  MappingGrid_SetPose(&mapping_pose);
  TestApp_ResetPoseHistory();
  TestApp_RecordPoseHistory(HAL_GetTick());
}

static maze_cell_t TestApp_GetNearestMazeCell(void)
{
  maze_cell_t cell;
  int32_t x = (mapping_pose.x_mm - MAZE_ORIGIN_X_MM) / MAZE_CELL_SIZE_MM;
  int32_t y = (mapping_pose.y_mm - MAZE_ORIGIN_Y_MM) / MAZE_CELL_SIZE_MM;

  if (mapping_pose.x_mm < MAZE_ORIGIN_X_MM)
  {
    x = 0L;
  }
  if (mapping_pose.y_mm < MAZE_ORIGIN_Y_MM)
  {
    y = 0L;
  }

  if (x < 0L)
  {
    x = 0L;
  }
  else if (x >= (int32_t)MAZE_GRID_CELLS)
  {
    x = (int32_t)MAZE_GRID_CELLS - 1L;
  }

  if (y < 0L)
  {
    y = 0L;
  }
  else if (y >= (int32_t)MAZE_GRID_CELLS)
  {
    y = (int32_t)MAZE_GRID_CELLS - 1L;
  }

  cell.x = (uint8_t)x;
  cell.y = (uint8_t)y;
  return cell;
}

static void TestApp_ResetNavObstacle(void)
{
  nav_front_min_mm = UINT16_MAX;
  nav_front_blocking_angle_cdeg = 0U;
  nav_front_update_tick_ms = 0U;
  nav_front_scan_min_mm = UINT16_MAX;
  nav_front_scan_angle_cdeg = 0U;
  nav_front_last_angle_cdeg = 0U;
  nav_front_last_distance_mm = 0U;
  nav_front_cluster_count = 0U;
  nav_front_cluster_open = false;
  TestApp_ResetNavBlockCandidate();
}

static void TestApp_UpdateNavObstacle(const LidarPoint_t *point, uint32_t now_ms)
{
  if ((point == NULL) || (point->distance_mm == 0U))
  {
    return;
  }

  if ((point->flags & LIDAR_POINT_FLAG_SCAN_START) != 0U)
  {
    TestApp_CommitNavFrontCluster(now_ms);
    nav_front_scan_min_mm = UINT16_MAX;
    nav_front_scan_angle_cdeg = 0U;
    nav_front_last_angle_cdeg = 0U;
    nav_front_last_distance_mm = 0U;
    nav_front_cluster_count = 0U;
    nav_front_cluster_open = false;
  }

  if (!TestApp_IsFrontLidarPoint(point->angle_cdeg))
  {
    return;
  }

  if (nav_front_cluster_open &&
      (TestApp_AngleDeltaAbsCdeg(point->angle_cdeg, nav_front_last_angle_cdeg) <= NAV_OBSTACLE_CLUSTER_ANGLE_CDEG) &&
      (AppAbs32((int32_t)point->distance_mm - (int32_t)nav_front_last_distance_mm) <= (int32_t)NAV_OBSTACLE_CLUSTER_RANGE_MM))
  {
    if (nav_front_cluster_count < 255U)
    {
      nav_front_cluster_count++;
    }
  }
  else
  {
    TestApp_CommitNavFrontCluster(now_ms);
    nav_front_cluster_count = 1U;
    nav_front_scan_min_mm = UINT16_MAX;
    nav_front_scan_angle_cdeg = 0U;
    nav_front_cluster_open = true;
  }

  nav_front_last_angle_cdeg = point->angle_cdeg;
  nav_front_last_distance_mm = point->distance_mm;
  if (point->distance_mm < nav_front_scan_min_mm)
  {
    nav_front_scan_min_mm = point->distance_mm;
    nav_front_scan_angle_cdeg = point->angle_cdeg;
  }
}

static void TestApp_CommitNavFrontCluster(uint32_t now_ms)
{
  if (nav_front_cluster_open &&
      (nav_front_cluster_count >= NAV_OBSTACLE_MIN_CLUSTER_POINTS))
  {
    nav_front_min_mm = nav_front_scan_min_mm;
    nav_front_blocking_angle_cdeg = nav_front_scan_angle_cdeg;
    nav_front_update_tick_ms = now_ms;
  }
}

static bool TestApp_HasFreshNavFrontSample(uint32_t now_ms)
{
  return ((nav_front_update_tick_ms != 0U) &&
          ((now_ms - nav_front_update_tick_ms) <= NAV_OBSTACLE_HOLD_MS));
}

static bool TestApp_IsNavFrontBlocked(uint32_t now_ms, uint16_t *out_distance_mm, uint16_t *out_angle_cdeg)
{
  uint16_t stop_distance_mm;

  if ((nav_front_min_mm == UINT16_MAX) || !TestApp_HasFreshNavFrontSample(now_ms))
  {
    return false;
  }

  stop_distance_mm = (uint16_t)(GetObstacleSafeDistanceMm() + NAV_OBSTACLE_STOP_MARGIN_MM);
  if (nav_front_min_mm > stop_distance_mm)
  {
    return false;
  }

  if (out_distance_mm != NULL)
  {
    *out_distance_mm = nav_front_min_mm;
  }
  if (out_angle_cdeg != NULL)
  {
    *out_angle_cdeg = nav_front_blocking_angle_cdeg;
  }
  return true;
}

static bool TestApp_ConfirmNavBlocked(uint32_t now_ms, uint16_t distance_mm, uint16_t angle_cdeg)
{
  if (!nav_block_candidate_active)
  {
    nav_block_candidate_active = true;
    nav_block_candidate_start_ms = now_ms;
    nav_block_candidate_distance_mm = distance_mm;
    nav_block_candidate_angle_cdeg = angle_cdeg;
    return false;
  }

  if (distance_mm < nav_block_candidate_distance_mm)
  {
    nav_block_candidate_distance_mm = distance_mm;
    nav_block_candidate_angle_cdeg = angle_cdeg;
  }

  return ((now_ms - nav_block_candidate_start_ms) >= NAV_OBSTACLE_CONFIRM_MS);
}

static void TestApp_ResetNavBlockCandidate(void)
{
  nav_block_candidate_active = false;
  nav_block_candidate_start_ms = 0U;
  nav_block_candidate_distance_mm = UINT16_MAX;
  nav_block_candidate_angle_cdeg = 0U;
}

static void TestApp_ClearNavDynamicWalls(void)
{
  memset(nav_blocked_vertical, 0, sizeof(nav_blocked_vertical));
  memset(nav_blocked_horizontal, 0, sizeof(nav_blocked_horizontal));
}

static void TestApp_MarkNavBlockedEdge(void)
{
  maze_cell_t from_cell = TestApp_GetNearestMazeCell();
  maze_cell_t to_cell;
  int32_t dx;
  int32_t dy;

  if (nav_path_index >= nav_path_length)
  {
    return;
  }

  to_cell = nav_path[nav_path_index];
  dx = (int32_t)to_cell.x - (int32_t)from_cell.x;
  dy = (int32_t)to_cell.y - (int32_t)from_cell.y;

  if ((AppAbs32(dx) + AppAbs32(dy)) != 1L)
  {
    uint8_t i;
    bool found_next_edge = false;

    for (i = 0U; (i + 1U) <= nav_path_index && (i + 1U) < nav_path_length; ++i)
    {
      if ((nav_path[i].x == from_cell.x) && (nav_path[i].y == from_cell.y))
      {
        to_cell = nav_path[i + 1U];
        dx = (int32_t)to_cell.x - (int32_t)from_cell.x;
        dy = (int32_t)to_cell.y - (int32_t)from_cell.y;
        found_next_edge = true;
        break;
      }
    }

    if (!found_next_edge)
    {
      if (nav_path_index == 0U)
      {
        return;
      }
      from_cell = nav_path[nav_path_index - 1U];
      dx = (int32_t)to_cell.x - (int32_t)from_cell.x;
      dy = (int32_t)to_cell.y - (int32_t)from_cell.y;
    }
  }

  if ((AppAbs32(dx) + AppAbs32(dy)) != 1L)
  {
    return;
  }

  if (dx > 0L)
  {
    nav_blocked_vertical[from_cell.x + 1U][from_cell.y] = true;
  }
  else if (dx < 0L)
  {
    nav_blocked_vertical[from_cell.x][from_cell.y] = true;
  }
  else if (dy > 0L)
  {
    nav_blocked_horizontal[from_cell.x][from_cell.y + 1U] = true;
  }
  else
  {
    nav_blocked_horizontal[from_cell.x][from_cell.y] = true;
  }
}

static bool TestApp_ReplanNavigationFromPose(const char *reason)
{
  char line[96];
  maze_cell_t current_cell = TestApp_GetNearestMazeCell();

  if (!TestApp_PlanPathAStar(&current_cell, &nav_goal_cell) || (nav_path_length == 0U))
  {
    TestApp_ClearNavDynamicWalls();
    if (!TestApp_PlanPathAStar(&current_cell, &nav_goal_cell) || (nav_path_length == 0U))
    {
      nav_state = NAV_STATE_FAILED;
      nav_active = false;
      MotorControl_Stop();
      (void)BluetoothControl_SendText("NAV FAIL replan-no-path\r\n");
      return false;
    }
    (void)BluetoothControl_SendText("NAV WARN dynamic-clear\r\n");
  }

  if (nav_path_length > 1U)
  {
    uint16_t current_distance = TestApp_MazeCellManhattan(&current_cell, &nav_goal_cell);
    uint16_t next_distance = TestApp_MazeCellManhattan(&nav_path[1], &nav_goal_cell);

    if (next_distance > current_distance)
    {
      TestApp_ClearNavDynamicWalls();
      if (TestApp_PlanPathAStar(&current_cell, &nav_goal_cell) && (nav_path_length > 0U))
      {
        (void)BluetoothControl_SendText("NAV WARN away-step-clear\r\n");
      }
    }
  }

  nav_start_cell = current_cell;
  nav_path_index = 0U;
  nav_turn_retry_count = 0U;
  TestApp_ResetNavObstacle();

  if (reason == NULL)
  {
    reason = "REPLAN";
  }

  (void)snprintf(
      line,
      sizeof(line),
      "NAV REPLAN %s from=%u,%u goal=%u,%u len=%u\r\n",
      reason,
      (unsigned int)current_cell.x,
      (unsigned int)current_cell.y,
      (unsigned int)nav_goal_cell.x,
      (unsigned int)nav_goal_cell.y,
      (unsigned int)nav_path_length);
  (void)BluetoothControl_SendText(line);

  if (nav_path_length <= 1U)
  {
    nav_state = NAV_STATE_DONE;
    nav_active = false;
    MotorControl_Stop();
    (void)BluetoothControl_SendText("NAV DONE\r\n");
    return true;
  }

  nav_path_index = 1U;
  TestApp_StartNavLeg();
  return true;
}

static void TestApp_UpdateNavTargetHeading(void)
{
  float heading_rad;
  int32_t dx_mm = nav_target_x_mm - mapping_pose.x_mm;
  int32_t dy_mm = nav_target_y_mm - mapping_pose.y_mm;

  if ((dx_mm == 0L) && (dy_mm == 0L))
  {
    return;
  }

  heading_rad = atan2f((float)dy_mm, (float)dx_mm);
  nav_target_heading_cdeg = NormalizeHeadingCdeg((int32_t)((heading_rad * 18000.0f) / MAPPING_PI));
}

static void TestApp_StartNavigation(void)
{
  char line[96];
  maze_cell_t current_cell;

  TestApp_StopAngleTurn(false);
  MotorControl_Stop();

  if (!mapping_active)
  {
    TestApp_StartMapping();
    TestApp_SetPoseToMazeCell(&nav_start_cell);
  }

  current_cell = TestApp_GetNearestMazeCell();
  nav_start_cell = current_cell;

  if (!TestApp_IsMazeCellValid(&nav_start_cell) || !TestApp_IsMazeCellValid(&nav_goal_cell))
  {
    nav_state = NAV_STATE_FAILED;
    nav_active = false;
    (void)BluetoothControl_SendText("NAV FAIL invalid-cell\r\n");
    return;
  }

  TestApp_ClearNavDynamicWalls();
  if (!TestApp_PlanPathAStar(&nav_start_cell, &nav_goal_cell) || (nav_path_length == 0U))
  {
    nav_state = NAV_STATE_FAILED;
    nav_active = false;
    (void)BluetoothControl_SendText("NAV FAIL no-path\r\n");
    return;
  }

  nav_active = true;
  nav_state = NAV_STATE_IDLE;
  nav_settle_next = NAV_SETTLE_NEXT_NONE;
  nav_settle_until_ms = 0U;
  nav_path_index = 0U;
  nav_turn_retry_count = 0U;
  TestApp_ResetNavObstacle();
  TestApp_ResetNavBlockCandidate();

  (void)snprintf(
      line,
      sizeof(line),
      "NAV RUN start=%u,%u goal=%u,%u len=%u\r\n",
      (unsigned int)nav_start_cell.x,
      (unsigned int)nav_start_cell.y,
      (unsigned int)nav_goal_cell.x,
      (unsigned int)nav_goal_cell.y,
      (unsigned int)nav_path_length);
  (void)BluetoothControl_SendText(line);

  if (nav_path_length <= 1U)
  {
    nav_state = NAV_STATE_DONE;
    nav_active = false;
    (void)BluetoothControl_SendText("NAV DONE already-at-goal\r\n");
    return;
  }

  nav_path_index = 1U;
  TestApp_StartNavLeg();
}

static void TestApp_StopNavigation(const char *reason)
{
  char line[48];

  if (!nav_active && (nav_state != NAV_STATE_TURNING) && (nav_state != NAV_STATE_DRIVING))
  {
    return;
  }

  nav_active = false;
  nav_state = NAV_STATE_IDLE;
  nav_settle_next = NAV_SETTLE_NEXT_NONE;
  nav_settle_until_ms = 0U;
  nav_path_length = 0U;
  nav_path_index = 0U;
  nav_turn_retry_count = 0U;
  TestApp_ResetNavObstacle();
  TestApp_ResetNavBlockCandidate();
  TestApp_StopAngleTurn(false);
  MotorControl_Stop();

  if (reason == NULL)
  {
    reason = "STOP";
  }

  (void)snprintf(line, sizeof(line), "NAV STOP %s\r\n", reason);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_UpdateNavigation(uint32_t now_ms)
{
  int32_t dx_mm;
  int32_t dy_mm;
  int32_t distance_sq;
  int32_t heading_error_cdeg;
  uint16_t drive_pwm;
  uint16_t obstacle_distance_mm;
  uint16_t obstacle_angle_cdeg;

  if (!nav_active)
  {
    return;
  }

  if (angle_turn_active)
  {
    return;
  }

  if (nav_state == NAV_STATE_SETTLING)
  {
    nav_settle_next_t next_action;

    if ((int32_t)(now_ms - nav_settle_until_ms) < 0L)
    {
      return;
    }

    next_action = nav_settle_next;
    nav_settle_next = NAV_SETTLE_NEXT_NONE;
    nav_settle_until_ms = 0U;

    if (next_action == NAV_SETTLE_NEXT_LEG)
    {
      TestApp_StartNavLeg();
      return;
    }

    if (next_action == NAV_SETTLE_NEXT_TURN)
    {
      nav_state = NAV_STATE_TURNING;
      TestApp_StartHeadingTurn(nav_target_heading_cdeg, 0U, "NAV-TRIM");
      return;
    }

    if (next_action == NAV_SETTLE_NEXT_DRIVE)
    {
      drive_pwm = ClampPwmPermille(GetDrivePwmPermille(), NAV_DRIVE_PWM_CAP);
      MotorControl_SetForward(drive_pwm);
      nav_state = NAV_STATE_DRIVING;
      return;
    }
    else
    {
      nav_state = NAV_STATE_IDLE;
      return;
    }
  }

  heading_error_cdeg = TestApp_SignedHeadingErrorCdeg(nav_target_heading_cdeg, mapping_pose.heading_cdeg);

  if (nav_state == NAV_STATE_TURNING)
  {
    if ((NAV_TURN_MAX_RETRIES > 0U) &&
        (AppAbs32(heading_error_cdeg) > NAV_TURN_HEADING_TOL_CDEG))
    {
      if (nav_turn_retry_count >= NAV_TURN_MAX_RETRIES)
      {
        nav_state = NAV_STATE_FAILED;
        nav_active = false;
        MotorControl_Stop();
        (void)BluetoothControl_SendText("NAV FAIL heading\r\n");
        return;
      }
      nav_turn_retry_count++;
      TestApp_StartHeadingTurn(nav_target_heading_cdeg, 0U, "NAV-RETRY");
      return;
    }

    if (TestApp_IsNavFrontBlocked(now_ms, &obstacle_distance_mm, &obstacle_angle_cdeg))
    {
      char line[80];
      MotorControl_Stop();
      if (!TestApp_ConfirmNavBlocked(now_ms, obstacle_distance_mm, obstacle_angle_cdeg))
      {
        return;
      }
      TestApp_MarkNavBlockedEdge();
      (void)snprintf(
          line,
          sizeof(line),
          "NAV BLOCKED front=%u safe=%u angle=%u action=replan\r\n",
          (unsigned int)obstacle_distance_mm,
          (unsigned int)GetObstacleSafeDistanceMm(),
          (unsigned int)obstacle_angle_cdeg);
      (void)BluetoothControl_SendText(line);
      TestApp_ResetNavBlockCandidate();
      (void)TestApp_ReplanNavigationFromPose("BLOCKED");
      return;
    }
    TestApp_ResetNavBlockCandidate();

    TestApp_StartNavSettle(now_ms, NAV_TURN_BRAKE_SETTLE_MS, NAV_SETTLE_NEXT_DRIVE);
    return;
  }

  if (nav_state != NAV_STATE_DRIVING)
  {
    return;
  }

  if (TestApp_IsNavFrontBlocked(now_ms, &obstacle_distance_mm, &obstacle_angle_cdeg))
  {
    char line[80];
    MotorControl_Stop();
    if (!TestApp_ConfirmNavBlocked(now_ms, obstacle_distance_mm, obstacle_angle_cdeg))
    {
      return;
    }
    TestApp_MarkNavBlockedEdge();
    (void)snprintf(
        line,
        sizeof(line),
        "NAV BLOCKED front=%u safe=%u angle=%u action=replan\r\n",
        (unsigned int)obstacle_distance_mm,
        (unsigned int)GetObstacleSafeDistanceMm(),
        (unsigned int)obstacle_angle_cdeg);
    (void)BluetoothControl_SendText(line);
    TestApp_ResetNavBlockCandidate();
    (void)TestApp_ReplanNavigationFromPose("BLOCKED");
    return;
  }
  TestApp_ResetNavBlockCandidate();

  dx_mm = nav_target_x_mm - mapping_pose.x_mm;
  dy_mm = nav_target_y_mm - mapping_pose.y_mm;
  distance_sq = (dx_mm * dx_mm) + (dy_mm * dy_mm);

  if (distance_sq <= (MAZE_TARGET_TOLERANCE_MM * MAZE_TARGET_TOLERANCE_MM))
  {
    MotorControl_Stop();
    if ((nav_path_index + 1U) >= nav_path_length)
    {
      nav_state = NAV_STATE_DONE;
      nav_active = false;
      (void)BluetoothControl_SendText("NAV DONE\r\n");
      return;
    }

    nav_path_index++;
    TestApp_StartNavSettle(now_ms, NAV_DRIVE_BRAKE_SETTLE_MS, NAV_SETTLE_NEXT_LEG);
    return;
  }

  if (AppAbs32(heading_error_cdeg) > NAV_DRIVE_HEADING_TOL_CDEG)
  {
    TestApp_StartNavSettle(now_ms, NAV_DRIVE_BRAKE_SETTLE_MS, NAV_SETTLE_NEXT_TURN);
    return;
  }

  drive_pwm = ClampPwmPermille(GetDrivePwmPermille(), NAV_DRIVE_PWM_CAP);
  MotorControl_SetForward(drive_pwm);
}

static void TestApp_StartNavSettle(uint32_t now_ms, uint32_t duration_ms, nav_settle_next_t next_action)
{
  MotorControl_Stop();
  nav_state = NAV_STATE_SETTLING;
  nav_settle_next = next_action;
  nav_settle_until_ms = now_ms + duration_ms;
}

static bool TestApp_PlanPathAStar(const maze_cell_t *start_cell, const maze_cell_t *goal_cell)
{
  bool vertical[MAZE_GRID_CELLS + 1U][MAZE_GRID_CELLS];
  bool horizontal[MAZE_GRID_CELLS][MAZE_GRID_CELLS + 1U];

  if (!TestApp_IsMazeCellValid(start_cell) || !TestApp_IsMazeCellValid(goal_cell))
  {
    return false;
  }

  TestApp_BuildMazeBounds(vertical, horizontal);
  return TestApp_SearchPathAStar(start_cell, goal_cell, vertical, horizontal);
}

static void TestApp_BuildMazeBounds(bool vertical[MAZE_GRID_CELLS + 1U][MAZE_GRID_CELLS],
                                    bool horizontal[MAZE_GRID_CELLS][MAZE_GRID_CELLS + 1U])
{
  uint8_t i;
  uint8_t j;

  memset(vertical, 0, sizeof(bool) * (MAZE_GRID_CELLS + 1U) * MAZE_GRID_CELLS);
  memset(horizontal, 0, sizeof(bool) * MAZE_GRID_CELLS * (MAZE_GRID_CELLS + 1U));

  for (j = 0U; j < MAZE_GRID_CELLS; ++j)
  {
    vertical[0U][j] = true;
    vertical[MAZE_GRID_CELLS][j] = true;
  }
  for (i = 0U; i < MAZE_GRID_CELLS; ++i)
  {
    horizontal[i][0U] = true;
    horizontal[i][MAZE_GRID_CELLS] = true;
  }
}

static bool TestApp_IsWallBetween(const bool vertical[MAZE_GRID_CELLS + 1U][MAZE_GRID_CELLS],
                                  const bool horizontal[MAZE_GRID_CELLS][MAZE_GRID_CELLS + 1U],
                                  uint8_t x,
                                  uint8_t y,
                                  int8_t dx,
                                  int8_t dy)
{
  if (dx > 0)
  {
    return (vertical[x + 1U][y] || nav_blocked_vertical[x + 1U][y]);
  }
  if (dx < 0)
  {
    return (vertical[x][y] || nav_blocked_vertical[x][y]);
  }
  if (dy > 0)
  {
    return (horizontal[x][y + 1U] || nav_blocked_horizontal[x][y + 1U]);
  }
  if (dy < 0)
  {
    return (horizontal[x][y] || nav_blocked_horizontal[x][y]);
  }
  return true;
}

static bool TestApp_SearchPathAStar(const maze_cell_t *start_cell,
                                    const maze_cell_t *goal_cell,
                                    const bool vertical[MAZE_GRID_CELLS + 1U][MAZE_GRID_CELLS],
                                    const bool horizontal[MAZE_GRID_CELLS][MAZE_GRID_CELLS + 1U])
{
  bool open[MAZE_GRID_CELLS * MAZE_GRID_CELLS];
  bool closed[MAZE_GRID_CELLS * MAZE_GRID_CELLS];
  uint16_t g_score[MAZE_GRID_CELLS * MAZE_GRID_CELLS];
  uint16_t f_score[MAZE_GRID_CELLS * MAZE_GRID_CELLS];
  int8_t parent[MAZE_GRID_CELLS * MAZE_GRID_CELLS];
  maze_cell_t reverse_path[MAZE_GRID_CELLS * MAZE_GRID_CELLS];
  const int8_t dir_x[4] = {1, -1, 0, 0};
  const int8_t dir_y[4] = {0, 0, 1, -1};
  uint8_t start_index = (uint8_t)((start_cell->y * MAZE_GRID_CELLS) + start_cell->x);
  uint8_t goal_index = (uint8_t)((goal_cell->y * MAZE_GRID_CELLS) + goal_cell->x);
  uint8_t i;
  uint8_t path_len = 0U;
  int8_t current;

  memset(open, 0, sizeof(open));
  memset(closed, 0, sizeof(closed));
  for (i = 0U; i < (MAZE_GRID_CELLS * MAZE_GRID_CELLS); ++i)
  {
    g_score[i] = UINT16_MAX;
    f_score[i] = UINT16_MAX;
    parent[i] = -1;
  }

  open[start_index] = true;
  g_score[start_index] = 0U;
  f_score[start_index] = (uint16_t)(AppAbs32((int32_t)start_cell->x - (int32_t)goal_cell->x) +
                                    AppAbs32((int32_t)start_cell->y - (int32_t)goal_cell->y));

  while (true)
  {
    uint8_t best_index = 0U;
    uint16_t best_score = UINT16_MAX;
    uint16_t best_goal_distance = UINT16_MAX;
    bool found = false;

    for (i = 0U; i < (MAZE_GRID_CELLS * MAZE_GRID_CELLS); ++i)
    {
      if (open[i])
      {
        maze_cell_t candidate;
        uint16_t candidate_goal_distance;

        candidate.x = (uint8_t)(i % MAZE_GRID_CELLS);
        candidate.y = (uint8_t)(i / MAZE_GRID_CELLS);
        candidate_goal_distance = TestApp_MazeCellManhattan(&candidate, goal_cell);

        if ((f_score[i] < best_score) ||
            ((f_score[i] == best_score) && (candidate_goal_distance < best_goal_distance)))
        {
          best_score = f_score[i];
          best_goal_distance = candidate_goal_distance;
          best_index = i;
          found = true;
        }
      }
    }

    if (!found)
    {
      nav_path_length = 0U;
      return false;
    }

    if (best_index == goal_index)
    {
      current = (int8_t)goal_index;
      while ((current >= 0) && (path_len < (MAZE_GRID_CELLS * MAZE_GRID_CELLS)))
      {
        reverse_path[path_len].x = (uint8_t)((uint8_t)current % MAZE_GRID_CELLS);
        reverse_path[path_len].y = (uint8_t)((uint8_t)current / MAZE_GRID_CELLS);
        path_len++;
        current = parent[(uint8_t)current];
      }

      nav_path_length = path_len;
      for (i = 0U; i < path_len; ++i)
      {
        nav_path[i] = reverse_path[(path_len - 1U) - i];
      }
      return true;
    }

    open[best_index] = false;
    closed[best_index] = true;

    for (i = 0U; i < 4U; ++i)
    {
      uint8_t cx = (uint8_t)(best_index % MAZE_GRID_CELLS);
      uint8_t cy = (uint8_t)(best_index / MAZE_GRID_CELLS);
      int16_t nx = (int16_t)cx + dir_x[i];
      int16_t ny = (int16_t)cy + dir_y[i];
      uint8_t neighbor_index;
      uint16_t tentative_g;
      uint16_t heuristic;

      if ((nx < 0) || (ny < 0) || (nx >= (int16_t)MAZE_GRID_CELLS) || (ny >= (int16_t)MAZE_GRID_CELLS))
      {
        continue;
      }

      if (TestApp_IsWallBetween(vertical, horizontal, cx, cy, dir_x[i], dir_y[i]))
      {
        continue;
      }

      neighbor_index = (uint8_t)(((uint8_t)ny * MAZE_GRID_CELLS) + (uint8_t)nx);
      if (closed[neighbor_index])
      {
        continue;
      }

      tentative_g = (uint16_t)(g_score[best_index] + 1U);
      if (!open[neighbor_index])
      {
        open[neighbor_index] = true;
      }
      else if (tentative_g >= g_score[neighbor_index])
      {
        continue;
      }

      parent[neighbor_index] = (int8_t)best_index;
      g_score[neighbor_index] = tentative_g;
      heuristic = (uint16_t)(AppAbs32(nx - (int32_t)goal_cell->x) +
                             AppAbs32(ny - (int32_t)goal_cell->y));
      f_score[neighbor_index] = (uint16_t)(tentative_g + heuristic);
    }
  }
}

static void TestApp_StartNavLeg(void)
{
  char line[96];
  maze_cell_t from;
  maze_cell_t target;
  int32_t step_x;
  int32_t step_y;
  bool heading_from_grid = true;

  if (!nav_active || (nav_path_index >= nav_path_length))
  {
    return;
  }

  from = (nav_path_index > 0U) ? nav_path[nav_path_index - 1U] : TestApp_GetNearestMazeCell();
  target = nav_path[nav_path_index];
  step_x = (int32_t)target.x - (int32_t)from.x;
  step_y = (int32_t)target.y - (int32_t)from.y;
  if (step_x > 0L)
  {
    step_x = 1L;
    step_y = 0L;
    nav_target_heading_cdeg = 0L;
  }
  else if (step_x < 0L)
  {
    step_x = -1L;
    step_y = 0L;
    nav_target_heading_cdeg = 18000L;
  }
  else if (step_y > 0L)
  {
    step_x = 0L;
    step_y = 1L;
    nav_target_heading_cdeg = 9000L;
  }
  else if (step_y < 0L)
  {
    step_x = 0L;
    step_y = -1L;
    nav_target_heading_cdeg = 27000L;
  }
  else
  {
    heading_from_grid = false;
  }

  if ((AppAbs32(step_x) + AppAbs32(step_y)) == 1L)
  {
    uint8_t lookahead_count = 1U;

    while ((lookahead_count < NAV_TARGET_LOOKAHEAD_CELLS) &&
           ((nav_path_index + 1U) < nav_path_length))
    {
      maze_cell_t current = nav_path[nav_path_index];
      maze_cell_t next = nav_path[nav_path_index + 1U];
      int32_t next_step_x = (int32_t)next.x - (int32_t)current.x;
      int32_t next_step_y = (int32_t)next.y - (int32_t)current.y;

      if (next_step_x > 0L)
      {
        next_step_x = 1L;
        next_step_y = 0L;
      }
      else if (next_step_x < 0L)
      {
        next_step_x = -1L;
        next_step_y = 0L;
      }
      else if (next_step_y > 0L)
      {
        next_step_x = 0L;
        next_step_y = 1L;
      }
      else if (next_step_y < 0L)
      {
        next_step_x = 0L;
        next_step_y = -1L;
      }

      if ((next_step_x != step_x) || (next_step_y != step_y))
      {
        break;
      }

      nav_path_index++;
      target = next;
      lookahead_count++;
    }
  }

  nav_target_x_mm = TestApp_MazeCellCenterX(target.x);
  nav_target_y_mm = TestApp_MazeCellCenterY(target.y);
  if (!heading_from_grid)
  {
    TestApp_UpdateNavTargetHeading();
  }

  nav_turn_retry_count = 0U;
  nav_state = NAV_STATE_TURNING;

  (void)snprintf(
      line,
      sizeof(line),
      "NAV LEG idx=%u cell=%u,%u target=%ld,%ld head=%ld\r\n",
      (unsigned int)nav_path_index,
      (unsigned int)target.x,
      (unsigned int)target.y,
      (long)nav_target_x_mm,
      (long)nav_target_y_mm,
      (long)nav_target_heading_cdeg);
  (void)BluetoothControl_SendText(line);

  if (AppAbs32(TestApp_SignedHeadingErrorCdeg(nav_target_heading_cdeg, mapping_pose.heading_cdeg)) <= NAV_TURN_HEADING_TOL_CDEG)
  {
    MotorControl_SetForward(ClampPwmPermille(GetDrivePwmPermille(), NAV_DRIVE_PWM_CAP));
    nav_state = NAV_STATE_DRIVING;
  }
  else
  {
    TestApp_StartHeadingTurn(nav_target_heading_cdeg, 0U, "NAV");
  }
}

static void TestApp_StartOdomDebug(void)
{
  uint32_t now = HAL_GetTick();

  odom_debug_active = true;
  odom_debug_seq = 0U;
  odom_debug_left_counts = 0L;
  odom_debug_right_counts = 0L;
  odom_debug_heading_cdeg = 0L;
  last_odom_debug_update_tick_ms = now;
  last_odom_debug_tx_tick_ms = 0U;

  (void)BluetoothControl_SendText("ODOM START format=ODOM seq=<n> lc=<left_counts> rc=<right_counts> avg=<counts> dist=<mm> heading=<cdeg> gz=<corrected_dps_x100> gb=<bias_dps_x100> k=<mm_x1000_per_count>\r\n");
}

static void TestApp_StopOdomDebug(void)
{
  if (!odom_debug_active)
  {
    return;
  }

  odom_debug_active = false;
  (void)BluetoothControl_SendText("ODOM STOP\r\n");
}

static void TestApp_UpdateOdomDebug(uint32_t now_ms, int16_t left_delta, int16_t right_delta)
{
  uint32_t delta_ms;

  if (!odom_debug_active)
  {
    last_odom_debug_update_tick_ms = now_ms;
    return;
  }

  if (last_odom_debug_update_tick_ms == 0U)
  {
    last_odom_debug_update_tick_ms = now_ms;
    return;
  }

  delta_ms = now_ms - last_odom_debug_update_tick_ms;
  last_odom_debug_update_tick_ms = now_ms;

  odom_debug_left_counts += (int32_t)left_delta;
  odom_debug_right_counts += (int32_t)right_delta;

  if (mpu_state.ready)
  {
    odom_debug_heading_cdeg += (gyro_z_corrected_dps_x100 * (int32_t)delta_ms) / 1000L;
  }
}

static void TestApp_StreamOdomDebug(void)
{
  char line[160];
  uint32_t now = HAL_GetTick();
  int32_t abs_left;
  int32_t abs_right;
  int32_t average_counts;
  int32_t distance_mm;

  if (!odom_debug_active)
  {
    return;
  }

  if ((now - last_odom_debug_tx_tick_ms) < ODOM_DEBUG_TX_INTERVAL_MS)
  {
    return;
  }
  last_odom_debug_tx_tick_ms = now;

  abs_left = AppAbs32(odom_debug_left_counts);
  abs_right = AppAbs32(odom_debug_right_counts);
  average_counts = (abs_left + abs_right) / 2L;
  distance_mm = (average_counts * MAPPING_ENCODER_MM_PER_COUNT_X1000) / 1000L;

  (void)snprintf(
      line,
      sizeof(line),
      "ODOM seq=%lu lc=%ld rc=%ld avg=%ld dist=%ld heading=%ld gz=%ld gb=%ld k=%ld\r\n",
      (unsigned long)odom_debug_seq++,
      (long)odom_debug_left_counts,
      (long)odom_debug_right_counts,
      (long)average_counts,
      (long)distance_mm,
      (long)odom_debug_heading_cdeg,
      (long)gyro_z_corrected_dps_x100,
      (long)gyro_z_bias_dps_x100,
      (long)MAPPING_ENCODER_MM_PER_COUNT_X1000);
  (void)BluetoothControl_SendText(line);
}

static uint16_t TestApp_ParseTurnDegrees(const char *text)
{
  uint32_t value = 0U;
  bool has_digit = false;

  if (text == NULL)
  {
    return ANGLE_TURN_DEFAULT_DEG;
  }

  while (*text != '\0')
  {
    if ((*text >= '0') && (*text <= '9'))
    {
      has_digit = true;
      value = (value * 10U) + (uint32_t)(*text - '0');
      if (value > ANGLE_TURN_MAX_DEG)
      {
        value = ANGLE_TURN_MAX_DEG;
        break;
      }
    }
    text++;
  }

  if (!has_digit)
  {
    value = ANGLE_TURN_DEFAULT_DEG;
  }
  else if (value < ANGLE_TURN_MIN_DEG)
  {
    value = ANGLE_TURN_MIN_DEG;
  }

  return (uint16_t)value;
}

static void TestApp_StartAngleTurn(int8_t direction, uint16_t degrees)
{
  char line[80];
  uint16_t turn_pwm;

  if (degrees < ANGLE_TURN_MIN_DEG)
  {
    degrees = ANGLE_TURN_MIN_DEG;
  }
  else if (degrees > ANGLE_TURN_MAX_DEG)
  {
    degrees = ANGLE_TURN_MAX_DEG;
  }

  angle_turn_active = true;
  angle_turn_direction = (direction < 0) ? -1 : 1;
  angle_turn_heading_target_valid = false;
  angle_turn_correction_count = 0U;
  angle_turn_target_cdeg = (int32_t)degrees * 100L;
  angle_turn_target_heading_cdeg = 0L;
  angle_turn_progress_cdeg = 0L;
  angle_turn_start_tick_ms = HAL_GetTick();
  angle_turn_last_tick_ms = angle_turn_start_tick_ms;

  turn_pwm = ClampPwmPermille(GetTurnPwmPermille(), ANGLE_TURN_PWM_CAP);
  if (angle_turn_direction < 0)
  {
    MotorControl_SetTurnLeft(turn_pwm);
  }
  else
  {
    MotorControl_SetTurnRight(turn_pwm);
  }

  (void)snprintf(
      line,
      sizeof(line),
      "TURN START dir=%c deg=%u target=%ld\r\n",
      (angle_turn_direction < 0) ? 'L' : 'R',
      (unsigned int)degrees,
      (long)angle_turn_target_cdeg);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_StartHeadingTurn(int32_t target_heading_cdeg, uint8_t correction_count, const char *reason)
{
  char line[96];
  int32_t error_cdeg;
  uint16_t degrees;
  int8_t direction;

  target_heading_cdeg = NormalizeHeadingCdeg(target_heading_cdeg);
  error_cdeg = TestApp_SignedHeadingErrorCdeg(target_heading_cdeg, mapping_pose.heading_cdeg);
  degrees = (uint16_t)((AppAbs32(error_cdeg) + 50L) / 100L);
  if (degrees < ANGLE_TURN_MIN_DEG)
  {
    degrees = ANGLE_TURN_MIN_DEG;
  }
  direction = (error_cdeg >= 0L) ? -1 : 1;

  TestApp_StartAngleTurn(direction, degrees);
  angle_turn_heading_target_valid = true;
  angle_turn_correction_count = correction_count;
  angle_turn_target_heading_cdeg = target_heading_cdeg;

  if (reason == NULL)
  {
    reason = "TARGET";
  }

  (void)snprintf(
      line,
      sizeof(line),
      "TURN HEADING %s target=%ld current=%ld err=%ld corr=%u\r\n",
      reason,
      (long)angle_turn_target_heading_cdeg,
      (long)mapping_pose.heading_cdeg,
      (long)error_cdeg,
      (unsigned int)angle_turn_correction_count);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_StopAngleTurn(bool completed)
{
  char line[80];
  bool heading_target_valid;
  uint8_t correction_count;
  int32_t target_heading_cdeg;
  int32_t heading_error_cdeg = 0L;

  if (!angle_turn_active)
  {
    return;
  }

  heading_target_valid = angle_turn_heading_target_valid;
  correction_count = angle_turn_correction_count;
  target_heading_cdeg = angle_turn_target_heading_cdeg;
  if (heading_target_valid)
  {
    heading_error_cdeg = TestApp_SignedHeadingErrorCdeg(target_heading_cdeg, mapping_pose.heading_cdeg);
  }

  angle_turn_active = false;
  angle_turn_direction = 0;
  angle_turn_heading_target_valid = false;
  MotorControl_Stop();

  if (completed &&
      heading_target_valid &&
      (AppAbs32(heading_error_cdeg) > ANGLE_TURN_CORRECTION_TOL_CDEG) &&
      (correction_count < ANGLE_TURN_MAX_CORRECTIONS))
  {
    TestApp_StartHeadingTurn(target_heading_cdeg, (uint8_t)(correction_count + 1U), "CORRECT");
    return;
  }

  (void)snprintf(
      line,
      sizeof(line),
      "TURN %s progress=%ld target=%ld herr=%ld\r\n",
      completed ? "DONE" : "STOP",
      (long)angle_turn_progress_cdeg,
      (long)angle_turn_target_cdeg,
      (long)heading_error_cdeg);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_UpdateAngleTurn(uint32_t now_ms)
{
  uint32_t delta_ms;
  int32_t delta_cdeg;
  int32_t remaining_cdeg;
  uint16_t turn_pwm;

  if (!angle_turn_active)
  {
    angle_turn_last_tick_ms = now_ms;
    return;
  }

  if (angle_turn_last_tick_ms == 0U)
  {
    angle_turn_last_tick_ms = now_ms;
    return;
  }

  delta_ms = now_ms - angle_turn_last_tick_ms;
  angle_turn_last_tick_ms = now_ms;
  delta_cdeg = (gyro_z_corrected_dps_x100 * (int32_t)delta_ms) / 1000L;
  angle_turn_progress_cdeg += AppAbs32(delta_cdeg);

  if (angle_turn_heading_target_valid)
  {
    remaining_cdeg = AppAbs32(TestApp_SignedHeadingErrorCdeg(
        angle_turn_target_heading_cdeg,
        mapping_pose.heading_cdeg));
  }
  else
  {
    remaining_cdeg = angle_turn_target_cdeg - angle_turn_progress_cdeg;
  }

  if (remaining_cdeg <= ANGLE_TURN_DONE_TOL_CDEG)
  {
    TestApp_StopAngleTurn(true);
    return;
  }

  if ((now_ms - angle_turn_start_tick_ms) >= ANGLE_TURN_TIMEOUT_MS)
  {
    TestApp_StopAngleTurn(false);
    return;
  }

  turn_pwm = ClampPwmPermille(GetTurnPwmPermille(), ANGLE_TURN_PWM_CAP);
  if ((remaining_cdeg <= ANGLE_TURN_FINE_ZONE_CDEG) && (turn_pwm > ANGLE_TURN_FINE_PWM))
  {
    turn_pwm = ANGLE_TURN_FINE_PWM;
  }
  else if ((remaining_cdeg <= ANGLE_TURN_SLOW_ZONE_CDEG) && (turn_pwm > ANGLE_TURN_SLOW_PWM))
  {
    turn_pwm = ANGLE_TURN_SLOW_PWM;
  }

  if (angle_turn_direction < 0)
  {
    MotorControl_SetTurnLeft(turn_pwm);
  }
  else
  {
    MotorControl_SetTurnRight(turn_pwm);
  }
}

static bool TestApp_IsFrontLidarPoint(uint16_t angle_cdeg)
{
  return ((angle_cdeg <= OBSTACLE_FRONT_SECTOR_CDEG) ||
          (angle_cdeg >= (uint16_t)(36000U - OBSTACLE_FRONT_SECTOR_CDEG)));
}

static int32_t TestApp_SignedHeadingErrorCdeg(int32_t target_cdeg, int32_t current_cdeg)
{
  int32_t error = NormalizeHeadingCdeg(target_cdeg) - NormalizeHeadingCdeg(current_cdeg);

  if (error > 18000L)
  {
    error -= 36000L;
  }
  else if (error < -18000L)
  {
    error += 36000L;
  }

  return error;
}

static void TestApp_StreamMap(void)
{
  char row_text[MAPPING_GRID_WIDTH_CELLS + 1U];
  char line[MAPPING_GRID_WIDTH_CELLS + 48U];
  uint32_t now = HAL_GetTick();
  uint32_t revision;

  if (!mapping_active)
  {
    return;
  }

  if ((now - last_map_stat_tx_tick_ms) >= MAP_STAT_TX_INTERVAL_MS)
  {
    last_map_stat_tx_tick_ms = now;
    TestApp_SendMapStat();
    TestApp_SendMapDiag();
  }

  if ((now - last_map_row_tx_tick_ms) < MAP_ROW_TX_INTERVAL_MS)
  {
    return;
  }

  last_map_row_tx_tick_ms = now;
  revision = MappingGrid_GetRevision();

  if (MappingGrid_FormatRow(next_map_tx_row, row_text, sizeof(row_text)))
  {
    (void)snprintf(
        line,
        sizeof(line),
        "MAP ROW y=%u rev=%lu data=%s\r\n",
        (unsigned int)next_map_tx_row,
        (unsigned long)revision,
        row_text);
    (void)BluetoothControl_SendText(line);
  }

  next_map_tx_row++;
  if (next_map_tx_row >= MAPPING_GRID_HEIGHT_CELLS)
  {
    next_map_tx_row = 0U;
  }
}

static void TestApp_StreamPose(void)
{
  char line[96];
  uint32_t now = HAL_GetTick();

  if (!mapping_active)
  {
    return;
  }

  if ((now - last_pose_tx_tick_ms) < POSE_TX_INTERVAL_MS)
  {
    return;
  }
  last_pose_tx_tick_ms = now;

  (void)snprintf(
      line,
      sizeof(line),
      "POSE %ld,%ld,%ld state=%s\r\n",
      (long)mapping_pose.x_mm,
      (long)mapping_pose.y_mm,
      (long)mapping_pose.heading_cdeg,
      TestApp_GetMotionStateText());
  (void)BluetoothControl_SendText(line);
}

static const char *TestApp_GetMotionStateText(void)
{
  MotorControlState_t motor_state = {0};

  if (angle_turn_active || (nav_state == NAV_STATE_TURNING))
  {
    return "TURNING";
  }

  if (!MotorControl_GetState(&motor_state))
  {
    return "UNKNOWN";
  }

  if (motor_state.forward_active || (motor_state.mode == 1U))
  {
    return "FORWARD";
  }

  if ((motor_state.mode == 2U) || (motor_state.mode == 3U))
  {
    return "TURNING";
  }

  return "STOPPED";
}

static void TestApp_RequestFullMapStream(void)
{
  next_map_tx_row = 0U;
  last_map_row_tx_tick_ms = 0U;
  TestApp_SendMapHeader(mapping_active ? "SNAP" : "IDLE");
  TestApp_SendMapStat();
  TestApp_SendMapDiag();
}

static void TestApp_SendMapHeader(const char *state)
{
  char line[96];

  if (state == NULL)
  {
    state = "INFO";
  }

  (void)snprintf(
      line,
      sizeof(line),
      "MAP %s w=%u h=%u cell=%umm rev=%lu\r\n",
      state,
      (unsigned int)MAPPING_GRID_WIDTH_CELLS,
      (unsigned int)MAPPING_GRID_HEIGHT_CELLS,
      (unsigned int)MAPPING_GRID_CELL_SIZE_MM,
      (unsigned long)MappingGrid_GetRevision());
  (void)BluetoothControl_SendText(line);
}

static void TestApp_SendMapStat(void)
{
  MappingGridStats_t stats;
  char line[192];

  if (!MappingGrid_GetStats(&stats))
  {
    return;
  }

  (void)snprintf(
      line,
      sizeof(line),
      "MAP STAT active=%u rev=%lu inserted=%lu rejected=%lu unknown=%u free=%u occupied=%u clipped=%lu drops=%lu pose=%ld,%ld,%ld\r\n",
      mapping_active ? 1U : 0U,
      (unsigned long)stats.revision,
      (unsigned long)stats.inserted_points,
      (unsigned long)stats.rejected_points,
      (unsigned int)stats.unknown_cells,
      (unsigned int)stats.free_cells,
      (unsigned int)stats.occupied_cells,
      (unsigned long)stats.clipped_rays,
      (unsigned long)LidarPipeline_GetPointQueueDrops(),
      (long)mapping_pose.x_mm,
      (long)mapping_pose.y_mm,
      (long)mapping_pose.heading_cdeg);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_SendMapDiag(void)
{
  char line[160];

  (void)snprintf(
      line,
      sizeof(line),
      "MAP DIAG scans=%lu/%lu/%lu short=%lu hold=%lu match=%lu/%lu miss=%lu\r\n",
      (unsigned long)mapping_diag.scans_started,
      (unsigned long)mapping_diag.scans_completed,
      (unsigned long)mapping_diag.scans_inserted,
      (unsigned long)mapping_diag.scans_short,
      (unsigned long)mapping_diag.scans_motion_held,
      (unsigned long)mapping_diag.scans_matched,
      (unsigned long)mapping_diag.scans_unmatched,
      (unsigned long)mapping_diag.pose_misses);
  (void)BluetoothControl_SendText(line);

  (void)snprintf(
      line,
      sizeof(line),
      "MAP PERF pts=%lu/%lu qrej=%lu ovf=%lu last=%u/%u used=%u score=%ld corr=%ld,%ld,%ld\r\n",
      (unsigned long)mapping_diag.points_buffered,
      (unsigned long)mapping_diag.points_inserted,
      (unsigned long)mapping_diag.points_quality_rejected,
      (unsigned long)mapping_diag.points_overflowed,
      (unsigned int)mapping_diag.last_scan_points,
      (unsigned int)mapping_diag.last_scan_inserted,
      (unsigned int)mapping_diag.last_match_used,
      (long)mapping_diag.last_match_score,
      (long)mapping_diag.last_match_dx_mm,
      (long)mapping_diag.last_match_dy_mm,
      (long)mapping_diag.last_match_dtheta_cdeg);
  (void)BluetoothControl_SendText(line);
}

static void TestApp_UpdateMotorSpeedFromAdc(void)
{
  MotorControlState_t motor_state = {0};
  uint16_t target_pwm;

  if (angle_turn_active || nav_active)
  {
    return;
  }

  if (!MotorControl_GetState(&motor_state))
  {
    return;
  }

  if (motor_state.mode == 1U)
  {
    target_pwm = GetDrivePwmPermille();
    if (PwmValueChanged(motor_state.duty_permille, target_pwm))
    {
      MotorControl_SetForward(target_pwm);
    }
  }
  else if (motor_state.mode == 2U)
  {
    target_pwm = GetTurnPwmPermille();
    if (PwmValueChanged(motor_state.duty_permille, target_pwm))
    {
      MotorControl_SetTurnLeft(target_pwm);
    }
  }
  else if (motor_state.mode == 3U)
  {
    target_pwm = GetTurnPwmPermille();
    if (PwmValueChanged(motor_state.duty_permille, target_pwm))
    {
      MotorControl_SetTurnRight(target_pwm);
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  TestApp_InitPeripherals();
  app_ready = true;
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(senseTask, StartSenseTask, osPriorityAboveNormal, 0, 512);
  senseTaskHandle = osThreadCreate(osThread(senseTask), NULL);

  osThreadDef(uiTask, StartUiTask, osPriorityBelowNormal, 0, 512);
  uiTaskHandle = osThreadCreate(osThread(uiTask), NULL);

  osThreadDef(btTask, StartBtTask, osPriorityNormal, 0, 256);
  btTaskHandle = osThreadCreate(osThread(btTask), NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 89;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  (void)argument;

  vTaskDelete(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSenseTask */
/**
  * @brief  Function implementing the senseTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartSenseTask */
void StartSenseTask(void const * argument)
{
  /* USER CODE BEGIN StartSenseTask */
  (void)argument;

  for(;;)
  {
    if (app_ready)
    {
      MotorControl_UpdateButtons();
      TestApp_UpdateSensors();
    }
    osDelay(5);
  }
  /* USER CODE END StartSenseTask */
}

/* USER CODE BEGIN Header_StartUiTask */
/**
  * @brief  Function implementing the uiTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUiTask */
void StartUiTask(void const * argument)
{
  /* USER CODE BEGIN StartUiTask */
  (void)argument;

  for(;;)
  {
    if (app_ready)
    {
      TestApp_PollButtons();
      TestApp_UpdateDisplay();
    }
    osDelay(20);
  }
  /* USER CODE END StartUiTask */
}

/* USER CODE BEGIN Header_StartBtTask */
/**
  * @brief  Function implementing the btTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBtTask */
void StartBtTask(void const * argument)
{
  /* USER CODE BEGIN StartBtTask */
  (void)argument;

  for(;;)
  {
    if (app_ready)
    {
      BluetoothControl_Update();
      TestApp_HandleBluetoothCommands();
      TestApp_StreamMap();
      TestApp_StreamPose();
      TestApp_StreamOdomDebug();
    }
    osDelay(20);
  }
  /* USER CODE END StartBtTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
