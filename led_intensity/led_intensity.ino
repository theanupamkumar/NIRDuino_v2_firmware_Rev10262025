// --- LIBRARIES ---
#include <Arduino.h>
#include "driver/gpio.h"
#include <NimBLEDevice.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Adafruit_MCP4728.h>
#include <Wire.h>
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include "driver/spi_master.h"
#include <CD74HC4067.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "rom/ets_sys.h" 

// --- DEBUG & CONFIGURATION ---
#define DEBUG_MODE
#define STORAGE_NAMESPACE "led_storage"
#define LED_INTENSITIES_KEY "led_intensities"
// #define DEBUG_BYPASS_BATTERY

// --- PIN DEFINITIONS (IMPORTANT: UPDATE THESE TO MATCH YOUR HARDWARE) ---
// RGB Status LED
const int PIN_LED_RED   = 14;
const int PIN_LED_GREEN = 16;
const int PIN_LED_BLUE  = 15;

// Analog MUX (CD74HC4067)
const int MUX_s0 = 19;
const int MUX_s1 = 20;
const int MUX_s2 = 17;
const int MUX_s3 = 18;
const int MUX_EN = 23; // MUX Enable Pin (Active LOW)

// ADC 1 Pins
const int ADS1256_1_CS_PIN    = 4;
const int ADS1256_1_DRDY_PIN  = 3;
const int ADS1256_1_RESET_PIN = 5;

// ADC 2 Pins
const int ADS1256_2_CS_PIN    = 7;
const int ADS1256_2_DRDY_PIN  = 6;
const int ADS1256_2_RESET_PIN = 8;

// ONLY USE BELOW-PINS INSIDE ADC FUNCTIONS WHEN YOU USE ESP-IDF BASEE LOW LEVEL API
#define esp_ADS1256_1_CS_PIN             GPIO_NUM_7
#define esp_ADS1256_1_DRDY_PIN           GPIO_NUM_6
#define esp_ADS1256_1_RESET_PIN          GPIO_NUM_8
#define esp_ADS1256_2_CS_PIN             GPIO_NUM_10
#define esp_ADS1256_2_DRDY_PIN           GPIO_NUM_9
#define esp_ADS1256_2_RESET_PIN          GPIO_NUM_17
#define esp_ADS1256_MOSI_PIN    GPIO_NUM_38
#define esp_ADS1256_MISO_PIN    GPIO_NUM_47
#define esp_ADS1256_CLK_PIN     GPIO_NUM_48
#define esp_MUX_EN                GPIO_NUM_13
// --- ADC Configuration Struct ---
typedef struct {
    spi_device_handle_t spi_handle;
    gpio_num_t cs_pin;
    gpio_num_t drdy_pin;
    gpio_num_t reset_pin;
    const char* name;
    SemaphoreHandle_t drdy_sem;
} ads1256_config_t;
// END OF ESP-IDF API PIN DEFINITION

// --- LOGGING MACROS (for ESP-IDF compatibility) ---
static const char *TAG = "NIRduino";
#ifdef DEBUG_MODE
  #define ESP_LOGI(tag, format, ...) Serial.printf("[%s][I] " format "\n", tag, ##__VA_ARGS__)
  #define ESP_LOGW(tag, format, ...) Serial.printf("[%s][W] " format "\n", tag, ##__VA_ARGS__)
  #define ESP_LOGE(tag, format, ...) Serial.printf("[%s][E] " format "\n", tag, ##__VA_ARGS__)
#else
  #define ESP_LOGI(tag, format, ...)
  #define ESP_LOGW(tag, format, ...)
  #define ESP_LOGE(tag, format, ...)
#endif

// --- BATTERY THRESHOLDS ---
#define LOW_BATT_THRESHOLD 20
#define CRITICAL_BATT_THRESHOLD 5

// --- LIBRARY OBJECTS ---
SFE_MAX1704X lipo(MAX1704X_MAX17043);
CD74HC4067 my_mux(MUX_s0, MUX_s1, MUX_s2, MUX_s3);
Adafruit_MCP4728 mcp;


// --- FREERTOS HANDLES ---
static SemaphoreHandle_t i2c_mutex;
static SemaphoreHandle_t drdy_semaphore1;
static SemaphoreHandle_t drdy_semaphore2;
static SemaphoreHandle_t adc_semaphore;
static SemaphoreHandle_t ble_semaphore;
QueueHandle_t storageQueue;

// --- GLOBAL VARIABLES ---
static bool deviceConnected = false;
static bool readDataBool = false;

// Data structure constants
const int NUM_SOURCES = 32 + 1; // 32 LEDs + 1 dark cycle
#define NUM_LEDS (NUM_SOURCES - 1)
const int DETECTORS_PER_SOURCE = 16;
#define SLOTS_PER_SOURCE (DETECTORS_PER_SOURCE + 1) // 16 detector values + 1 timestamp
const int PACKET_SIZE = NUM_SOURCES * SLOTS_PER_SOURCE;

// Data buffers
int data_packet[PACKET_SIZE];
int dataPacketToSend[PACKET_SIZE];

// Default LED intensities
static uint8_t ledIntensities[NUM_LEDS] = {
    100, 100, 100, 100, 100, 100, 100, 100,
    100, 100, 100, 100, 100, 100, 100, 100,
    50,  50,  50,  50,  50,  50,  50,  50,
    50,  50,  50,  50,  50,  50,  50,  50
};

// --- BLE UUIDs and Device Name ---
#define DEVICE_NAME               "BBOL NIRDuino (Nano ESP32)"
#define FNIRS_SERVICE_UUID        "938548e6-c655-11ea-87d0-0242ac130003"
#define DATA_CHARACTERISTIC_UUID  "77539407-6493-4b89-985f-baaf4c0f8d86"
#define DATA_CHARACTERISTIC_UUID2 "513b630c-e5fd-45b5-a678-bb2835d6c1d2"
#define LED_CHARACTERISTIC_UUID   "19B10001-E8F2-537E-4F6C-D104768A1213"

// --- GLOBAL BLE POINTERS ---
NimBLEServer* pServer = NULL;
NimBLECharacteristic* pDataCharacteristic = NULL;
NimBLECharacteristic* pDataCharacteristic2 = NULL;
NimBLECharacteristic* pLEDCharacteristic = NULL;

// --- ADC CONFIGURATION ---
#define SAMPLING_FREQUENCY_HZ 279
#define SAMPLING_PERIOD_US (1000000 / SAMPLING_FREQUENCY_HZ)
static esp_timer_handle_t periodic_timer;

static ads1256_config_t adc1_config;
static ads1256_config_t adc2_config;

// --- FUNCTION PROTOTYPES ---
void initBLE();
void initGPIO();
void initDACs();
void initBatteryFuelGauge();
static void initStorageTask();
esp_err_t initLEDIntensitiesNVS();
void init_adc_system();
void start_adc_sampling();
void stop_adc_sampling();
void ble_broadcast_task(void *pvParameters);
void led_battery_status_task(void *pvParameters);
static void adc_sampling_task(void* arg);
void storage_task(void *pvParameters);
static void setLedColor(int red, int green, int blue);

// --- LED INTENSITY RANGE TEST FUNCTIONS ---
void process_led_calibration_config(uint8_t* sources, uint8_t* detectors, size_t count);
static void set_led_intensity_for_calibration(uint8_t source, int intensity_8bit);
static float read_detector_voltage(uint8_t detector);
static void send_calibration_result_ble(uint8_t source, int intensity);

int rangeOfValues[] = {105, 115, 125, 135, 145, 155,   165,   175,   185,   195,   205,   215,   225,   235,   245,   255};
static const size_t rangeOfValuesCount = sizeof(rangeOfValues) / sizeof(rangeOfValues[0]);

// --- ADC VOLTAGE CONVERSION ---
#define ADS1256_VREF 2.5  // Internal reference voltage of ADS1256
#define ADS1256_FSR 5.0   // Full Scale Range (2 * VREF)
#define ADS1256_MAX_CODE 8388608.0 // Resolution (2^23)
#define ADS1256_PGA 1.0 // Assuming Gain = 1


// --- ISR HANDLERS ---
// --- NEW: ISR Handler for DRDY signal ---
static void IRAM_ATTR drdy_isr_handler(void* arg) {
    SemaphoreHandle_t drdy_sem = (SemaphoreHandle_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(drdy_sem, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


static void IRAM_ATTR timer_callback(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(adc_semaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}


/**
 * @brief --- NEW: Helper function to print the current 32-value ledIntensities array
 */
void printLEDIntensities() {
    ESP_LOGI(TAG, "--- Current LED Intensities ---");
    
    // Print "Long" channels (Sources 0-15)
    Serial.print("Long (0-15):  ");
    for (int i = 0; i < 16; i++) {
        Serial.printf("%d", ledIntensities[i]);
        if (i < 15) Serial.print(", ");
    }
    Serial.println(); // new line

    // Print "Short" channels (Sources 16-31)
    Serial.print("Short (16-31): ");
    for (int i = 16; i < 32; i++) {
        Serial.printf("%d", ledIntensities[i]);
        if (i < 31) Serial.print(", ");
    }
    Serial.println(); // final newline
    ESP_LOGI(TAG, "---------------------------------");
}


//======================================================================================
// SETUP FUNCTION - Main Initialization
//======================================================================================
void setup() {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.begin(115200);
    // while(!Serial); // Wait for serial connection
    Serial.println("Starting NIRduino application...");
    // --- Create Semaphores and Mutexes ---
    i2c_mutex = xSemaphoreCreateMutex();
    ble_semaphore = xSemaphoreCreateBinary();
    if (i2c_mutex == NULL || ble_semaphore == NULL) {
        Serial.println("Failed to create mutex/semaphore");
        while(1);
    }
    // // --- Initialize Hardware and System Components ---
    initGPIO();

    // Init I2c
    pinMode(A4, INPUT_PULLUP);  // SDA
    pinMode(A5, INPUT_PULLUP);  // SCL
    Wire.begin();
    
    initDACs();
    initBatteryFuelGauge();
     // Initialize NVS and load LED intensities
    esp_err_t err = initLEDIntensitiesNVS();
    if (err != ESP_OK) {
        Serial.print("Failed to initialize NVS: ");
        Serial.println(esp_err_to_name(err));
        // Continue anyway with default values
    }
    initStorageTask();
    initBLE();
    init_adc_system();
    // // --- Create and Start FreeRTOS Tasks ---
    #ifndef DEBUG_BYPASS_BATTERY
    xTaskCreate(led_battery_status_task, "led_battery_task", 2048, NULL, 2, NULL);
    #else
    Serial.println("--- WARNING: DEBUG_BYPASS_BATTERY enabled. led_battery_status_task disabled. ---");
    #endif
    xTaskCreate(ble_broadcast_task, "ble_broadcast_task", 4096, NULL, 4, NULL);
    Serial.println("NIRduino initialization complete. Ready.");

    //  const size_t test_count = 2;
    //             uint8_t test_sources[test_count] = {0, 16};
    //             uint8_t test_detectors[test_count] = {1, 1};
    //             // Call the new function (no longer sends channel_types)
    //             // process_led_calibration_config(sources, detectors, count);
    //             process_led_calibration_config(test_sources, test_detectors, test_count);
}

//======================================================================================
// LOOP FUNCTION - Runs as the lowest priority task
//======================================================================================
void loop() {
    // The main loop is kept clear as all functionality is handled by FreeRTOS tasks.
    // A delay prevents this task from starving other tasks.
    vTaskDelay(pdMS_TO_TICKS(1000));
}


//======================================================================================
// INITIALIZATION FUNCTIONS
//======================================================================================

void initGPIO() {
   Serial.println("Initializing GPIO...");
   pinMode(PIN_LED_RED, OUTPUT);
   pinMode(PIN_LED_GREEN, OUTPUT);
   pinMode(PIN_LED_BLUE, OUTPUT);

   // Set boot-up color to YELLOW
   setLedColor(255, 255, 0);

   // Initialize MUX pins
   pinMode(MUX_s0, OUTPUT);
   pinMode(MUX_s1, OUTPUT);
   pinMode(MUX_s2, OUTPUT);
   pinMode(MUX_s3, OUTPUT);
   pinMode(MUX_EN, OUTPUT);
   digitalWrite(MUX_EN, HIGH); // Disable MUX initially

   // Initialize ADC control pins
   pinMode(ADS1256_1_CS_PIN, OUTPUT);
   pinMode(ADS1256_1_DRDY_PIN, INPUT);
   pinMode(ADS1256_1_RESET_PIN, OUTPUT);
   pinMode(ADS1256_2_CS_PIN, OUTPUT);
   pinMode(ADS1256_2_DRDY_PIN, INPUT);
   pinMode(ADS1256_2_RESET_PIN, OUTPUT);
   
   digitalWrite(ADS1256_1_CS_PIN, HIGH);
   digitalWrite(ADS1256_2_CS_PIN, HIGH);
}

void initDACs(){
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
    if (!mcp.begin(0x60)) {
        Serial.println("Failed to find MCP4728 chip. Freezing.");
        while (1) { vTaskDelay(10); }
    }
    xSemaphoreGive(i2c_mutex);
    Serial.println("DAC init successful!");
  }
}

void initBatteryFuelGauge(){
    if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        if (lipo.begin() == false) {
            Serial.println("MAX1704x not detected. Please check wiring.");
            #ifndef DEBUG_BYPASS_BATTERY
            Serial.println("Freezing.");
            while (1) { vTaskDelay(10); }
            #else
            Serial.println("--- WARNING: DEBUG_BYPASS_BATTERY enabled. Continuing without battery gauge. ---");
            #endif
        } else {
            Serial.println("MAX1704x Fuel Gauge detected.");
            lipo.quickStart();
            lipo.setThreshold(20); // Set alert threshold to 20%.
        }
        xSemaphoreGive(i2c_mutex);
    }
}


static void initStorageTask(void) {
    Serial.println("Creating storage task and queue...");
    
    // Create queue for storage operations
    storageQueue = xQueueCreate(5, sizeof(int));
    if (storageQueue == NULL) {
        Serial.println("Failed to create storage queue");
        return;
    }
    
    // Create storage task with low priority
    BaseType_t result = xTaskCreate(storage_task, "storage_task", 4096, NULL, 1, NULL);
    if (result != pdPASS) {
         Serial.println("Failed to create storage task");
    } else {
        #ifdef DEBUG_MODE
         Serial.println("Storage task created successfully");
        #endif
    }
}


//======================================================================================
// NVS-FUNCTIONS
//======================================================================================

/**
 * Check if LED intensities are stored in NVS
 * Returns true if data exists, false otherwise
 */
bool areLEDIntensitiesInNVS(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    size_t required_size = 0;
    
    Serial.println("Checking NVS for LED intensities...");
    
    // Open NVS handle
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        Serial.println("Error in opening NVS handle!");
        return false;
    }
    
    // Check if the blob exists by getting its size
    err = nvs_get_blob(my_handle, LED_INTENSITIES_KEY, NULL, &required_size);
    nvs_close(my_handle);
    
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        Serial.println("LED intensities not found in NVS");
        return false;
    } else if (err == ESP_OK && required_size == sizeof(ledIntensities)) {
        Serial.println("LED intensities found in NVS");
        return true;
    } else {
        Serial.println("Error or size mismatch checking NVS!");
        return false;
    }
}

/**
 * Save LED intensities array to NVS
 * Returns ESP_OK on success, error code otherwise
 */
esp_err_t saveLEDIntensitiesToNVS(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    
    Serial.println("Saving LED intensities to NVS...");
    
    // Open NVS handle
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        Serial.println("Error opening NVS handle!");
        return err;
    }
    
    // Write LED intensities as a blob
    err = nvs_set_blob(my_handle, LED_INTENSITIES_KEY, ledIntensities, sizeof(ledIntensities));
    if (err != ESP_OK) {
        Serial.println("Error writing LED intensities!");
        nvs_close(my_handle);
        return err;
    }
    
    // Commit changes
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        Serial.println("Error committing LED intensities!");
    } else {
        Serial.println("LED intensities saved successfully");
    }
    
    nvs_close(my_handle);
    return err;
}

/**
 * Load LED intensities array from NVS
 * Returns ESP_OK on success, error code otherwise
 */
esp_err_t loadLEDIntensitiesFromNVS(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;
    size_t required_size = sizeof(ledIntensities);
    
    Serial.println("Loading LED intensities from NVS...");
    
    // Open NVS handle
    err = nvs_open(STORAGE_NAMESPACE, NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        Serial.println("Error opening NVS handle!");
        return err;
    }
    
    // Read LED intensities blob
    err = nvs_get_blob(my_handle, LED_INTENSITIES_KEY, ledIntensities, &required_size);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            Serial.println("LED intensities not found in NVS!");
        } else {
            Serial.println("Error reading LED intensities!");
        }
    } else if (required_size != sizeof(ledIntensities)) {
        Serial.println("Size mismatch");
        err = ESP_ERR_INVALID_SIZE;
    } else {
        Serial.println("LED intensities loaded successfully");
        
        #ifdef DEBUG_MODE
        // // Print loaded values for debugging
        // Serial.println("Loaded LED intensities:");
        // for (int i = 0; i < NUM_LEDS; i++) {
        //     Serial.println("LED[%d] = %d", i, ledIntensities[i]);
        // }
        #endif
    }
    
    nvs_close(my_handle);
    return err;
}

/**
 * Initialize default LED intensities
 */
void initDefaultLEDIntensities(void)
{
    Serial.println("Initializing default LED intensities");
    int defaultIntensities[NUM_SOURCES-1] = {100, 100, 100, 100,
                              100, 100, 100, 100,
                              100, 100, 100, 100,
                              100, 100, 100, 100,
                              50, 50, 50, 50,
                              50, 50, 50, 50,
                              50, 50, 50, 50,
                              50, 50, 50, 50
                              };
    memcpy(ledIntensities, defaultIntensities, sizeof(ledIntensities));
}

/**
 * Initialize NVS and LED intensities
 * Call this during system initialization (in app_main)
 */
esp_err_t initLEDIntensitiesNVS(void)
{
    esp_err_t err;
    
    Serial.println("Initializing NVS and LED intensities...");
    
    // Initialize NVS
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("NVS partition was truncated and needs to be erased");
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        Serial.println("Error initializing NVS!");
        return err;
    }
    
    // Check if LED intensities exist in NVS and load them
    if (areLEDIntensitiesInNVS()) {
        err = loadLEDIntensitiesFromNVS();
        if (err != ESP_OK) {
            Serial.println("Failed to load LED intensities from NVS, using defaults");
            initDefaultLEDIntensities();
        }
    } else {
        Serial.println("No LED intensities found in NVS, using default values");
        initDefaultLEDIntensities();
        // Save default values to NVS for next boot
        saveLEDIntensitiesToNVS();
    }
    
    return ESP_OK;
}

/**
 * @brief --- NEW: Send current LED intensities over BLE
 * This function is now fully implemented.
 */
static void sendLEDIntensities(void)
{
    if (deviceConnected && pDataCharacteristic != nullptr) {
        ESP_LOGI(TAG, "Sending all %d calibrated LED intensities over BLE...", NUM_LEDS);
        // We send the raw 32-byte array. The app knows how to parse this.
        pDataCharacteristic->notify(ledIntensities, NUM_LEDS);
    } else {
        ESP_LOGW(TAG, "Cannot send intensities, no client connected.");
    }
}


/**
 * @brief --- NEW: Reads a single detector and converts the raw ADC value to volts.
 * @param detector The detector to read (1-16)
 * @return The measured voltage as a float.
 */
static float read_detector_voltage(uint8_t detector) {
    if (detector < 1 || detector > 16) {
        ESP_LOGE(TAG, "Error: Invalid detector number %u", detector);
        return 0.0;
    }

    ads1256_config_t* adc;
    uint8_t adc_channel;

    // --- !!! --- FINAL "CLEAN" MAPPING --- !!! ---
    // This mapping assumes to be 0-indexed.
    // Detector 1 -> ADC1, AIN0 (PD1)
    // Detector 8 -> ADC1, AIN7 (PD8)
    // Detector 9 -> ADC2, AIN0 (PD9)
    // Detector 16 -> ADC2, AIN7 (PD16)
    
    if (detector <= 8) {
        // Detector 1-8 -> ADC1, AIN0-7
        adc = &adc1_config;
        uint8_t remapped_channel = detector - 1; // Map 1-8 to 0-7
        // MUX codes: {0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78}
        adc_channel = (remapped_channel << 4) | 0x08; // (0<<4)|8=0x08 (AIN0), (7<<4)|8=0x78 (AIN7)
    } else {
        // Detector 9-16 -> ADC2, AIN0-7
        adc = &adc2_config;
        uint8_t remapped_channel = (detector - 8) - 1; // Map 9-16 to 0-7
        adc_channel = (remapped_channel << 4) | 0x08; // (0<<4)|8=0x08 (AIN0), (7<<4)|8=0x78 (AIN7)
    }
   
    int32_t raw_adc_val = ADS1256_read_channel(adc, adc_channel);

    if (raw_adc_val == -1) {
        ESP_LOGE(TAG, "Error reading detector %u (%s, MUX 0x%X)", detector, adc->name, adc_channel);
        return -1.0;
    }

    float voltage = (float)raw_adc_val * (ADS1256_FSR / ADS1256_MAX_CODE) / ADS1256_PGA;
    return voltage;
}

/**
 * @brief --- NEW: Sets a specific LED source to a specific intensity for calibration.
 * This function is modeled on set_source_state and uses the SINGLE-DAC-channel logic.
 * @param source The source to turn on (0-31)
 * @param intensity_8bit The 8-bit intensity value to test (0-255)
 */
static void set_led_intensity_for_calibration(uint8_t source, int intensity_8bit) {
    // Convert 8-bit intensity (0-255) to 12-bit DAC value (0-4095)
    float voltage_dac = (intensity_8bit / 255.0) * 4095.0;
    int dac_value = (int)voltage_dac;

    // Case for dark current measurement (all LEDs off)
    if (source == 32) {
        gpio_set_level(esp_MUX_EN, 1); // Disable MUX
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
            mcp.setChannelValue(MCP4728_CHANNEL_A, 0); // Turn off DAC A
            xSemaphoreGive(i2c_mutex); 
        }
    }
    else {
        gpio_set_level(esp_MUX_EN, 0); // Enable MUX

        // Map Source 0-15 ("long") and 16-31 ("short") to MUX pins 0-15
        int muxPin = (source < 16) ? source : source - 16;
        my_mux.channel(muxPin);

        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
            // All intensity is controlled by CHANNEL_A, per the schematic
            mcp.setChannelValue(MCP4728_CHANNEL_A, dac_value); 
            xSemaphoreGive(i2c_mutex);
        }
        
        // Delay to let the LED and photodiode stabilize
        esp_rom_delay_us(1500); 
    }
}

/**
 * @brief --- NEW: Processes the LED intensity calibration configuration.
 * @param sources Array of source IDs (0-31)
 * @param detectors Array of detector IDs (1-16)
 * @param count The number of elements in each array
 */
void process_led_calibration_config(uint8_t* sources, uint8_t* detectors, size_t count) {
    ESP_LOGI(TAG, "--- STARTING LED CALIBRATION for %u pairs ---", (unsigned int)count);
    
    if (readDataBool) {
        ESP_LOGW(TAG, "Calibration Warning: Stopping data acquisition first.");
        stop_adc_sampling();
        readDataBool = false;
    }

    for (size_t i = 0; i < count; i++) {
        uint8_t s = sources[i];
        uint8_t d = detectors[i];
        
        // Infer channel type from source number for logging
        const char* type_str = (s < 16) ? "Long" : "Short";
        
        ESP_LOGI(TAG, "\nCalibrating Pair %u/%u: (Source %u [Type: %s], Detector %u)", 
            (unsigned int)i + 1, (unsigned int)count, s, type_str, d);
        
        // Collect all valid intensities, don't stop at the first match
        int valid_intensities[/* max = */ 64];  // adjust if your range can exceed 64
        size_t valid_count = 0;
        bool out_of_range_detected = false;

        bool found = false;
        // Step 1: sweep and save intensities
        for (size_t j = 0; j < rangeOfValuesCount; j++) {
            int test_intensity = rangeOfValues[j];

            set_led_intensity_for_calibration(s, test_intensity);
            vTaskDelay(pdMS_TO_TICKS(5)); 
            float voltage = read_detector_voltage(d);

            ESP_LOGI(TAG, "  Try Intensity %d -> Read %.3f V", test_intensity, voltage);

            if (voltage >= 0.4f && voltage <= 4.0f) {
                // Record as valid candidate
                if (valid_count < (sizeof(valid_intensities)/sizeof(valid_intensities[0]))) {
                    valid_intensities[valid_count++] = test_intensity;
                } else {
                    ESP_LOGW(TAG, "  Skipping additional valid intensities (buffer full)");
                }
            }
            else {
                 out_of_range_detected = true;
            }
        }
        // Step 2: Decide intensity
        if (out_of_range_detected) {
            ledIntensities[s] = 0;
            ESP_LOGW(TAG, "  WARNING: Out-of-range voltages detected. Setting Source %u intensity to 0.", s);
        } 
        else if (valid_count == 0) {
            ledIntensities[s] = 0;
            ESP_LOGW(TAG, "  FAILED: No valid intensity found for Source %u in the given range.", s);
        } 
        else {
            // rangeOfValues is usually sorted, but be safe: simple insertion sort (no stdlib needed)
            for (size_t a = 1; a < valid_count; a++) {
                int key = valid_intensities[a];
                size_t b = a;
                while (b > 0 && valid_intensities[b - 1] > key) {
                    valid_intensities[b] = valid_intensities[b - 1];
                    b--;
                }
                valid_intensities[b] = key;
            }

            // Pick median (lower-middle for even count)
            size_t median_idx = (valid_count - 1) / 2;
            int median_intensity = valid_intensities[median_idx];

            ledIntensities[s] = median_intensity;
            ESP_LOGI(TAG, "  SUCCESS: %zu valid values found. Median intensity for Source %u = %d",
                     valid_count, s, median_intensity);
        }
    }

    set_led_intensity_for_calibration(32, 0); 
    ESP_LOGI(TAG, "--- LED INTENSITY CALIBRATION FINISHED ---");

    // Send the entire LED INTENSITIES array back to the app
    sendLEDIntensities();
    
    int cmd = 1;
    if (xQueueSend(storageQueue, &cmd, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue NVS save after calibration");
    }
}


//======================================================================================
// BLE CALLBACKS AND INITIALIZATION
//======================================================================================
class MyServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        Serial.println("Client Connected");
        // Log the negotiated MTU size. This is crucial for debugging data transmission.
        Serial.println("Negotiated MTU (below in bytes)");
        Serial.println(connInfo.getMTU());
        deviceConnected = true;
        readDataBool = false;
        // Set to GREEN to indicate a successful connection.
        setLedColor(0, 255, 0); // Solid Green
    };

    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        Serial.println("Client Disconnected - reason(below): ");
        Serial.println(reason);
        deviceConnected = false;
        // Set to CYAN/TEAL to indicate ready to connect again.
        setLedColor(0, 255, 255); // Solid Cyan/Teal
        NimBLEDevice::startAdvertising();
    }
} myServerCallbacks;

class LEDCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
        std::string rxValue = pCharacteristic->getValue();
        if (rxValue.length() == 0) {
             Serial.println("LEDCallback: Received empty payload");
            return;
        }

        // The first byte is the command, the rest is the data payload
        uint8_t commandCode = rxValue[0];

        #ifdef DEBUG_MODE
         Serial.println("LEDCallback: Received command code and below bytes of data as follows below");
         Serial.println(commandCode);
         Serial.println(rxValue.length() - 1);
        #endif

        // Update LED intensities if data is provided
        if (rxValue.length() > 1 && commandCode != 23) {
            // Determine how many LED values we actually received
            size_t numLedsReceived = rxValue.length() - 1;
            // Determine the maximum number of LEDs we can update in our array
            size_t numLedsToUpdate = std::min(numLedsReceived, (size_t)NUM_LEDS);

            // CORRECT WAY: Iterate through the payload and assign values one by one.
            // This properly handles the conversion from an 8-bit byte to a 32-bit int.
            for (size_t i = 0; i < numLedsToUpdate; ++i) {
                // The LED intensity data starts at the second byte (index 1) of the payload
                ledIntensities[i] = (uint8_t)rxValue[i + 1];
            }
        
            Serial.print("LED Intensities:\n");

            // Print first 16 LEDs on one line
            for (int i = 0; i < 16; i++) {
            Serial.print(i + 1);
            Serial.print("=");
            Serial.print(ledIntensities[i]);
            if (i < 15) Serial.print(",\t");
            }
            Serial.println(); // new line after first 16

            // Print next 16 LEDs on second line
            for (int i = 16; i < 32; i++) {
            Serial.print(i + 1);
            Serial.print("=");
            Serial.print(ledIntensities[i]);
            if (i < 31) Serial.print(",\t");
            }
            Serial.println(); // final newline

        }

        // Handle commands (This part remains the same)
        switch (commandCode) {
            case 1: // START DATA ACQUISITION
                // #ifdef DEBUG_MODE
                Serial.println("Command: START DATA ACQUISITION");
                // #endif
                readDataBool = true; 
                // Queue async save operation to avoid blocking BLE
                {
                    int cmd = 1;
                    if (xQueueSend(storageQueue, &cmd, 0) != pdTRUE) {
                        Serial.println("Failed to queue save operation");
                    }
                }
                start_adc_sampling();
                sendBatteryLevel();
                break;

            case 3: // STOP DATA ACQUISITION
                Serial.println("Command: STOP DATA ACQUISITION");
                readDataBool = false;
                
                if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                    // Turn off DAC
                    mcp.setChannelValue(MCP4728_CHANNEL_A, 0);
                    xSemaphoreGive(i2c_mutex);
                }
                
                // Queue async save operation
                {
                    int cmd = 1;
                    if (xQueueSend(storageQueue, &cmd, 0) != pdTRUE) {
                        Serial.println("Failed to queue save operation");
                    }
                }
                stop_adc_sampling();
                sendBatteryLevel();
                break;

            case 9: // SEND BATTERY LEVEL
                sendBatteryLevel();
                break;

            case 11: // RETURN DESIRED LED Intensity levels
                // automaticLEDIntensityAdjustment()
                // sendLEDIntensities();
                // source/detector voltageR > 0.4 and less than <4.0V
                break;
            
            case 23: // PROCESS LED INTENSITY CALIBRATION CONFIG
              {
                ESP_LOGI(TAG, "Command: PROCESS LED CALIBRATION CONFIG");
                if (rxValue.length() < 2) {
                    ESP_LOGE(TAG, "Calibration Error: Payload too short (missing count)");
                    break;
                }

                // First byte after command is the count
                int count = rxValue[1];

                Serial.println("rx0, rx1, count value are below:");
                Serial.println(rxValue[0]);
                Serial.println(rxValue[1]);
                Serial.println(count);

                // 1 byte for cmd, 1 byte for count, 2 * count bytes for data (sources + detectors)
                size_t expected_length = 2 + (2 * count); 

                if (rxValue.length() != expected_length) {
                    ESP_LOGE(TAG, "Calibration Error: Payload length mismatch. Expected %u, Got %u",
                        (unsigned int)expected_length, (unsigned int)rxValue.length());
                    break;
                }

                if (count == 0) {
                     ESP_LOGI(TAG, "Calibration Info: Received 0 configurations.");
                     break;
                }

                // Create pointers to the start of each array within the payload
                // Payload structure: [cmd] [count] [s0...sn-1] [d0...dn-1]
                uint8_t* sources = (uint8_t*)&rxValue[2];
                uint8_t* detectors = (uint8_t*)&rxValue[2 + count];

                // Call the new function (no longer sends channel_types)
                process_led_calibration_config(sources, detectors, count);
                break;
              }

            default:
                Serial.println("LEDCallback: Unknown command code received");
                Serial.println(commandCode);
                break;
        }
    }
}ledCallbacks;


void initBLE() {
    Serial.println("Initializing BLE...");
    NimBLEDevice::init(DEVICE_NAME);
    NimBLEDevice::setMTU(517); // Set max MTU size for large data packets

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(&myServerCallbacks);

    NimBLEService *pfNIRSService = pServer->createService(FNIRS_SERVICE_UUID);

    pDataCharacteristic = pfNIRSService->createCharacteristic(
                                   DATA_CHARACTERISTIC_UUID,
                                   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                   NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);
    
    pDataCharacteristic2 = pfNIRSService->createCharacteristic(
                                   DATA_CHARACTERISTIC_UUID2,
                                   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE |
                                   NIMBLE_PROPERTY::NOTIFY | NIMBLE_PROPERTY::INDICATE);

    pLEDCharacteristic = pfNIRSService->createCharacteristic(
                                   LED_CHARACTERISTIC_UUID,
                                   NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE); 
    
    pLEDCharacteristic->setCallbacks(&ledCallbacks);

    pfNIRSService->start();

    NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
    NimBLEAdvertisementData advert;
    advert.setFlags(0x06);
    advert.setName(DEVICE_NAME);
    advert.addServiceUUID(FNIRS_SERVICE_UUID);
    
    pAdvertising->setAdvertisementData(advert);
    pAdvertising->enableScanResponse(false);
    pAdvertising->start();

    // Ready to conect - BLE Initialized
    setLedColor(0, 255, 255); // Solid Cyan/Teal
    Serial.println("BLE initialized and advertising started");
}


//======================================================================================
// ADC CONTROL AND SAMPLING
//======================================================================================

// --- Low-Level SPI Functions (Unchanged) ---
static void ADS1256_cmd(spi_device_handle_t spi, const uint8_t cmd) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &cmd;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void ADS1256_write_reg(spi_device_handle_t spi, const uint8_t reg, const uint8_t data) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 24;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = 0x50 | reg;
    t.tx_data[1] = 0x00;
    t.tx_data[2] = data;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

static void ADS1256_read_data(spi_device_handle_t spi, uint8_t* buffer) {
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 24;
    t.rx_buffer = buffer;
    ret = spi_device_polling_transmit(spi, &t);
    assert(ret == ESP_OK);
}

// --- High-Level ADS1256 Functions ---
static void ADS1256_init(ads1256_config_t* adc) {
    Serial.print("Initializing ");
    Serial.print(adc->name);
    gpio_set_direction(adc->reset_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(adc->reset_pin, 1); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(adc->reset_pin, 0); vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(adc->reset_pin, 1); vTaskDelay(pdMS_TO_TICKS(10));

    // Configure DRDY pin and attach ISR
    gpio_set_direction(adc->drdy_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(adc->drdy_pin, GPIO_FLOATING); // Use pullup for stability
    gpio_set_intr_type(adc->drdy_pin, GPIO_INTR_ANYEDGE); // Trigger on falling edge
    gpio_isr_handler_add(adc->drdy_pin, drdy_isr_handler, (void*)adc->drdy_sem);

    gpio_set_level(adc->cs_pin, 0);
    xSemaphoreTake(adc->drdy_sem, pdMS_TO_TICKS(100)); // Wait for DRDY using semaphore
    ADS1256_cmd(adc->spi_handle, 0xFE); // RESET
    vTaskDelay(pdMS_TO_TICKS(10));

    ADS1256_write_reg(adc->spi_handle, 0x00, 0x01); // STATUS
    ADS1256_write_reg(adc->spi_handle, 0x02, 0x00); // ADCON
    ADS1256_write_reg(adc->spi_handle, 0x03, 0xE0); // DRATE
    ADS1256_cmd(adc->spi_handle, 0xF0); // SELFCAL
    xSemaphoreTake(adc->drdy_sem, pdMS_TO_TICKS(100)); // Wait for calibration to finish
    gpio_set_level(adc->cs_pin, 1);
    Serial.print("Initialized and calibrated: ");
    Serial.println(adc->name);
}

static int32_t ADS1256_read_channel(ads1256_config_t* adc, uint8_t mux_setting) {
    int32_t adc_val = 0;
    uint8_t read_buffer[3];
    gpio_set_level(adc->cs_pin, 0);

    // Wait for the previous conversion to finish.
    if (xSemaphoreTake(adc->drdy_sem, pdMS_TO_TICKS(5)) != pdTRUE) {
        Serial.println("DRDY timeout before MUX write!");
        Serial.print(adc->name);
        gpio_set_level(adc->cs_pin, 1);
        return -1; // Indicate an error
    }

    ADS1256_write_reg(adc->spi_handle, 0x01, mux_setting);
    ADS1256_cmd(adc->spi_handle, 0xFC); // SYNC
    ADS1256_cmd(adc->spi_handle, 0x00); // WAKEUP

    // Wait for the new conversion to complete.
    if (xSemaphoreTake(adc->drdy_sem, pdMS_TO_TICKS(5)) != pdTRUE) {
        Serial.println("DRDY timeout after WAKEUP!");
        Serial.print(adc->name);
        gpio_set_level(adc->cs_pin, 1);
        return -1;
    }

    ADS1256_cmd(adc->spi_handle, 0x01); // RDATA
    ADS1256_read_data(adc->spi_handle, read_buffer);
    gpio_set_level(adc->cs_pin, 1);

    adc_val = ((int32_t)read_buffer[0] << 16) | ((int32_t)read_buffer[1] << 8) | (read_buffer[2]);
    if (adc_val & 0x800000) { adc_val |= 0xFF000000; }
    return adc_val;
}

/**
 * @brief Encapsulates all ADC hardware and task initialization.
 */
void init_adc_system(void) {
    Serial.println("Initializing ADC system...");
    esp_err_t ret;
    
    // --- 1. Create Semaphore ---
    adc_semaphore = xSemaphoreCreateBinary();
    if (adc_semaphore == NULL) {
        Serial.println("Failed to create semaphore");
        return;
    }
    drdy_semaphore1 = xSemaphoreCreateBinary(); // Create new semaphores
    drdy_semaphore2 = xSemaphoreCreateBinary();

    // Install GPIO ISR service, required for pin interrupts
    gpio_install_isr_service(0);

    // --- 2. Configure Hardware ---
    gpio_reset_pin(esp_ADS1256_1_CS_PIN);
    gpio_set_direction(esp_ADS1256_1_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(esp_ADS1256_1_CS_PIN, 1);
    gpio_reset_pin(esp_ADS1256_2_CS_PIN);
    gpio_set_direction(esp_ADS1256_2_CS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(esp_ADS1256_2_CS_PIN, 1);

    spi_bus_config_t buscfg = {
        .mosi_io_num = esp_ADS1256_MOSI_PIN,
        .miso_io_num = esp_ADS1256_MISO_PIN,
        .sclk_io_num = esp_ADS1256_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    // Initialize without DMA as it was faster for this use case
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, 0);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg = {
        .mode = 1,
        .clock_speed_hz = 1800 * 1000, // Matching the clock speed from your app_main
        .spics_io_num = -1,
        .queue_size = 7,
    };
    spi_device_handle_t shared_spi_handle;
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &shared_spi_handle);
    ESP_ERROR_CHECK(ret);

    // --- 3. Initialize ADC Configs ---
    adc1_config = (ads1256_config_t){ .spi_handle = shared_spi_handle, .cs_pin = esp_ADS1256_1_CS_PIN, .drdy_pin = esp_ADS1256_1_DRDY_PIN, .reset_pin = esp_ADS1256_1_RESET_PIN, .name = "ADC1", .drdy_sem = drdy_semaphore1 };
    adc2_config = (ads1256_config_t){ .spi_handle = shared_spi_handle, .cs_pin = esp_ADS1256_2_CS_PIN, .drdy_pin = esp_ADS1256_2_DRDY_PIN, .reset_pin = esp_ADS1256_2_RESET_PIN, .name = "ADC2", .drdy_sem = drdy_semaphore2 };
    
    ADS1256_init(&adc1_config);
    ADS1256_init(&adc2_config);
    
    // --- 4. Create and Start Periodic Timer ---
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "adc_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &periodic_timer));
    // ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, SAMPLING_PERIOD_US));
    
    // --- 5. Create Dedicated ADC Task on Core 1 ---
    xTaskCreatePinnedToCore(
        adc_sampling_task,   // Function to implement the task
        "adc_sampling_task", // Name of the task
        4096,                // Stack size in words
        NULL,                // Task input parameter
        5,                   // Priority of the task
        NULL,                // Task handle
        1                    // Core where the task should run
    );

    Serial.println("ADC Initialization complete");
}


void start_adc_sampling(void) {
    if (periodic_timer == NULL) {
        Serial.println("Timer not initialized. Call init_adc_system() first.");
        return;
    }
    esp_err_t err = esp_timer_start_periodic(periodic_timer, SAMPLING_PERIOD_US);
    if (err == ESP_OK) {
         Serial.print("ADC sampling timer started (in Hz): "); 
         Serial.println(SAMPLING_FREQUENCY_HZ);
    } else {
         Serial.print("Failed to start ADC timer: "); 
         Serial.println(esp_err_to_name(err));
    }
}

/**
 * @brief Stops the periodic ADC sampling timer.
 */
void stop_adc_sampling(void) {
    if (periodic_timer == NULL) {
        Serial.println("Timer not initialized.");
        return;
    }
    esp_err_t err = esp_timer_stop(periodic_timer);
    if (err == ESP_OK) {
        Serial.println("ADC sampling timer stopped.");
    } else {
        Serial.println("Failed to stop ADC timer!");
    }
}

static void sendBatteryLevel(){
  #ifdef DEBUG_MODE
  Serial.println("Sending battery level over BLE!");
  #endif

  int batteryLevel = 0;

  #ifndef DEBUG_BYPASS_BATTERY
  if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
  batteryLevel = lipo.getSOC();
  xSemaphoreGive(i2c_mutex);
  }
  #else
  batteryLevel = 100; // Send dummy 100% if battery is bypassed
  #endif

  #ifdef DEBUG_MODE
  Serial.print("Battery level: ");
  Serial.println(batteryLevel);
  #endif 
  
  if (pDataCharacteristic != NULL) {
    // NimBLE uses notify() with data directly, no setValue() method
    bool success = pDataCharacteristic->notify((uint8_t*)&batteryLevel, sizeof(batteryLevel));
    if (!success) {
      Serial.println("Failed to send battery level notification");
    }
  }
}


//======================================================================================
// APPLICATION LOGIC & TASKS
//======================================================================================

static void setLedColor(int red, int green, int blue) {
    // Assumes common-anode setup
    digitalWrite(PIN_LED_RED,   red > 0 ? LOW : HIGH);
    digitalWrite(PIN_LED_GREEN, green > 0 ? LOW : HIGH);
    digitalWrite(PIN_LED_BLUE,  blue > 0 ? LOW : HIGH);
}

static int getLEDIntensity(int sourceNumber){
    float voltage = (ledIntensities[sourceNumber]/255.0)*4095.0;
    return (int) voltage;
}

// --- Placeholder functions for your application logic ---
static void set_source_state(int sourceNumber) {
    // TODO: Implement your logic to turn on the correct LED/source
    // This will be called at the start of each measurement cycle.
    // For example: turn_on_led(sourceNumber);
     // Case for dark current measurement (all LEDs off)
    gpio_set_level(esp_MUX_EN, 1);

    if (sourceNumber == 32) {
        // Disable MUX (active low, so set HIGH)
        gpio_set_level(esp_MUX_EN, 1);

        // Take the mutex before using the I2C bus
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        
        // Set DAC output to 0 to turn off any active LED
        mcp.setChannelValue(MCP4728_CHANNEL_A, 0);

        // Give the mutex back immediately after the I2C command is sent
        xSemaphoreGive(i2c_mutex); 
        }
    }
    else {
        // Enable MUX for LED sources (active low, so set LOW)
        gpio_set_level(esp_MUX_EN, 0);

        int muxPin = (sourceNumber < 16) ? sourceNumber : sourceNumber - 16;
        
        // Set MUX to the correct channel
        my_mux.channel(muxPin);

        // Take the mutex before using the I2C bus
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
        
            mcp.setChannelValue(MCP4728_CHANNEL_A, getLEDIntensity(sourceNumber));

            // Give the mutex back immediately after the I2C command is sent
            xSemaphoreGive(i2c_mutex);

        }

        // Add a small, blocking delay to let the DAC voltage stabilize before ADC reading.
        // esp_rom_delay_us is used for short, precise delays inside a real-time task.
        esp_rom_delay_us(1500);
    }
}
/**
 * @brief Dedicated task for sampling ADCs when triggered by the timer.
 * @param arg Not used.
 */
static void adc_sampling_task(void* arg) {
    // Arrays to hold the raw ADC values for one complete cycle
    int32_t adc1_values[8];
    int32_t adc2_values[8];

    // Error tracking
    static int adc1_error_count = 0;
    static int adc2_error_count = 0;
    static int total_reads = 0;

    // The current light source to be activated. Static to persist across calls.
    static int sourceNumber = 0;
    
    // Timestamp of the last measurement cycle, in milliseconds.
    // Static to persist across calls.
    static uint32_t last_cycle_timestamp_ms = 0;

    // Multiplexer settings for the 8 channels of the ADS1256
    uint8_t mux[8] = {0x08, 0x18, 0x28, 0x38, 0x48, 0x58, 0x68, 0x78};

    Serial.println("ADC sampling task started");

    while (1) {
        // Wait for the semaphore from the timer ISR. This is CPU efficient.
        if (xSemaphoreTake(adc_semaphore, portMAX_DELAY) == pdTRUE) {
            uint64_t cycle_start_time = esp_timer_get_time();

            // 1. Activate the correct light source for this measurement cycle
            set_source_state(sourceNumber);
           
            // 2. Read all 16 channels (8 from each ADC) as quickly as possible
            // Add error checking to ADC reads
            for (int i = 0; i < 8; i++) {
                // Read ADC1 with error checking
                total_reads++;
                int32_t adc1_reading = ADS1256_read_channel(&adc1_config, mux[i]);
                if (adc1_reading == -1) {
                    adc1_error_count++;
                    adc1_values[i] = 0; // Use 0 instead of -1
                    // if (adc1_error_count % 10 == 0) { // Log every 10th error to avoid spam
                    //     Serial.print("ADC1 errors: ");
                    //     Serial.print(adc1_error_count);
                    //     Serial.print("/");
                    //     Serial.print(total_reads);
                    //     Serial.print(" (");
                    //     Serial.print((adc1_error_count * 100.0) / total_reads);
                    //     Serial.print("%) on channel ");
                    //     Serial.println(i);
                    // }
                } else {
                    adc1_values[i] = adc1_reading;
                }
                
                // Read ADC2 with error checking
                total_reads++;
                int32_t adc2_reading = ADS1256_read_channel(&adc2_config, mux[i]);
                if (adc2_reading == -1) {
                    adc2_error_count++;
                    adc2_values[i] = 0; // Use 0 instead of -1
                    // if (adc2_error_count % 10 == 0) { // Log every 10th error to avoid spam
                    //     Serial.print("ADC2 errors: ");
                    //     Serial.print(adc2_error_count);
                    //     Serial.print("/");
                    //     Serial.print(total_reads);
                    //     Serial.print(" (");
                    //     Serial.print((adc2_error_count * 100.0) / total_reads);
                    //     Serial.print("%) on channel ");
                    //     Serial.println(i);
                    // }
                } else {
                    adc2_values[i] = adc2_reading;
                }
            }

            /*----test code below - only uncommnet to map the app detectors accordingly----*/
            /*-----------------------------------------------------------------------------*/
            // Create a unique pattern for ADC1 (Detectors 9-16)
            // This is the *hardware* order (AIN0, AIN1, ...)
            // adc1_values[0] = 1000; // AIN0
            // adc1_values[1] = 0000; // AIN1
            // adc1_values[2] = 0000; // AIN2
            // adc1_values[3] = 0000; // AIN3
            // adc1_values[4] = 0000; // AIN4
            // adc1_values[5] = 0000; // AIN5
            // adc1_values[6] = 0000; // AIN6
            // adc1_values[7] = 0000; // AIN7

            // // Create a unique pattern for ADC2 (Detectors 9-16)
            // // This is the *hardware* order (AIN0, AIN1, ...)
            // adc2_values[0] = 2000; // AIN0
            // adc2_values[1] = 0000; // AIN1
            // adc2_values[2] = 0000; // AIN2
            // adc2_values[3] = 0000; // AIN3
            // adc2_values[4] = 0000; // AIN4
            // adc2_values[5] = 0000; // AIN5
            // adc2_values[6] = 0000; // AIN6
            // adc2_values[7] = 0000; // AIN7
            /*---------------------------------end of test code-----------------------------*/

            // 3. Store the collected data, remapping channels to match Arduino implementation.
            // Arduino storage order: AIN1->AIN7 (Detectors 1-7), then AIN0 (Detector 8).
            // Current read order: AIN0->AIN7 stored in adcX_values[0] through adcX_values[7].
            int base_index = sourceNumber * SLOTS_PER_SOURCE;

            memcpy(&data_packet[base_index], &adc1_values[1], 7 * sizeof(int32_t));
            // Copy AIN0 (from adc1_values[0]) to the 8th slot.
            data_packet[base_index + 7] = adc1_values[0];

            // --- Remap and store ADC2 data ---
            // Copy AIN1-AIN7 (from adc2_values[1]..[7]) to the next 7 slots (detectors 9-15).
            memcpy(&data_packet[base_index + 8], &adc2_values[1], 7 * sizeof(int32_t));
            // Copy AIN0 (from adc2_values[0]) to the 16th slot (detector 16).
            data_packet[base_index + 15] = adc2_values[0];

            // 4. Calculate and store the time interval since the last measurement.
            // uint32_t current_time_ms = (uint32_t)(esp_timer_get_time() / 1000);
            uint32_t current_time_ms = (uint32_t)(millis());
            
            if (last_cycle_timestamp_ms == 0) {
                last_cycle_timestamp_ms = current_time_ms;
            }

            uint32_t interval = current_time_ms - last_cycle_timestamp_ms;

            if(interval > 20){
                interval = 0;
            }

            data_packet[base_index + DETECTORS_PER_SOURCE] = interval;
            
            last_cycle_timestamp_ms = current_time_ms;

            // #ifdef DEBUG_MODE
            uint64_t cycle_duration_us = esp_timer_get_time() - cycle_start_time;
            // // Print results for debugging purposes for the current source
            // Serial.print("Source: ");
            // Serial.print(sourceNumber);
            // Serial.print(" | ADC1: ");
            // for (int i = 0; i < 8; i++) {
            //   Serial.print(adc1_values[i]);
            //   Serial.print(" ");
            // }

            // Serial.print("\n            | ADC2: ");
            // for (int i = 0; i < 8; i++) {
            //   Serial.print(adc2_values[i]);
            //   Serial.print(" ");
            // }

            // Serial.print("| Stored Interval: ");
            // Serial.print(interval);
            // Serial.print(" ms | Cycle Time: ");
            // Serial.print(cycle_duration_us);
            // Serial.println(" us\n");
            // #endif

            // 5. Advance the source number. If a full round is complete,
            // copy the data to the send buffer and reset. This matches the Arduino logic.
            if (sourceNumber == 32) { // After processing source 32 (the 33rd cycle)
                memcpy(dataPacketToSend, data_packet, sizeof(data_packet));
                xSemaphoreGive(ble_semaphore);
                sourceNumber = 0; // Reset for the next full round
            } else {
                sourceNumber++; // Advance to the next source
            }
        }
    }
}

/**
 * Storage task for async NVS operations
 * This runs at low priority to avoid blocking BLE operations
 */
void storage_task(void *pvParameters) {
    int command;
    Serial.println("Storage task started");
    
    for (;;) {
        // Wait for commands from the queue
        if (xQueueReceive(storageQueue, &command, portMAX_DELAY)) {
            #ifdef DEBUG_MODE
            Serial.println("Storage Task: Received command");
            #endif
            
            switch (command) {
                case 1: // Save LED intensities
                    {
                        esp_err_t err = saveLEDIntensitiesToNVS();
                        if (err == ESP_OK) {
                            Serial.println("Storage Task: LED intensities saved successfully");
                        } else {
                            Serial.println("Storage Task: Failed to save LED intensities");
                        }
                    }
                    break;
                    
                default:
                    Serial.println("Storage Task: Unknown command");
                    break;
            }
        }
    }
}


static void sendDataViaBLE(NimBLECharacteristic* pCharacteristic, uint8_t* data, size_t size) {
   if (deviceConnected && pCharacteristic != nullptr) {
        if (!pCharacteristic->notify(data, size)) {
            Serial.print("Failed to send notification. Size in bytes: ");
            Serial.println(size);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void ble_broadcast_task(void *pvParameters) {
    Serial.println("BLE broadcast task started.");
    const int CHUNK_1_ELEMENTS = 17 * 7 + 1;
    const int CHUNK_2_ELEMENTS = 17 * 5 + 1;
    int32_t dataPacketSnippet[CHUNK_1_ELEMENTS];

    while(1) {
        if(xSemaphoreTake(ble_semaphore, portMAX_DELAY) == pdTRUE) {
            if (readDataBool && deviceConnected) {
                // Serial.println("Data packet ready. Starting broadcast...");
                
                dataPacketSnippet[0] = 1;
                memcpy(&dataPacketSnippet[1], &dataPacketToSend[0], (CHUNK_1_ELEMENTS - 1) * sizeof(int32_t));
                sendDataViaBLE(pDataCharacteristic, (uint8_t*)dataPacketSnippet, CHUNK_1_ELEMENTS * sizeof(int32_t));
                
                dataPacketSnippet[0] = 2;
                memcpy(&dataPacketSnippet[1], &dataPacketToSend[7 * 17], (CHUNK_1_ELEMENTS - 1) * sizeof(int32_t));
                sendDataViaBLE(pDataCharacteristic2, (uint8_t*)dataPacketSnippet, CHUNK_1_ELEMENTS * sizeof(int32_t));

                dataPacketSnippet[0] = 3;
                memcpy(&dataPacketSnippet[1], &dataPacketToSend[14 * 17], (CHUNK_1_ELEMENTS - 1) * sizeof(int32_t));
                sendDataViaBLE(pDataCharacteristic2, (uint8_t*)dataPacketSnippet, CHUNK_1_ELEMENTS * sizeof(int32_t));

                dataPacketSnippet[0] = 4;
                memcpy(&dataPacketSnippet[1], &dataPacketToSend[21 * 17], (CHUNK_1_ELEMENTS - 1) * sizeof(int32_t));
                sendDataViaBLE(pDataCharacteristic2, (uint8_t*)dataPacketSnippet, CHUNK_1_ELEMENTS * sizeof(int32_t));

                dataPacketSnippet[0] = 5;
                memcpy(&dataPacketSnippet[1], &dataPacketToSend[28 * 17], (CHUNK_2_ELEMENTS - 1) * sizeof(int32_t));
                sendDataViaBLE(pDataCharacteristic, (uint8_t*)dataPacketSnippet, CHUNK_2_ELEMENTS * sizeof(int32_t));
                // Serial.println("Full data packet sent.");
            }
        }
    }
}

void led_battery_status_task(void *pvParameters) {
    Serial.println("LED Battery Status task started.");
    vTaskDelay(pdMS_TO_TICKS(5000));

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (!readDataBool) continue;

        int batteryLevel = 0;
        if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            batteryLevel = lipo.getSOC();
            xSemaphoreGive(i2c_mutex);
        } else {
            continue;
        }

        if (batteryLevel <= CRITICAL_BATT_THRESHOLD) {
            Serial.print("Battery critically low Halting: ");
            Serial.println(batteryLevel);
            readDataBool = false;
            stop_adc_sampling();
            if (xSemaphoreTake(i2c_mutex, portMAX_DELAY) == pdTRUE) {
                mcp.setChannelValue(MCP4728_CHANNEL_A, 0);
                xSemaphoreGive(i2c_mutex);
            }
            while (1) { // Blocking pulse
                setLedColor(255, 0, 0); vTaskDelay(pdMS_TO_TICKS(400));
                setLedColor(0, 0, 0); vTaskDelay(pdMS_TO_TICKS(400));
            }
        } else if (batteryLevel <= LOW_BATT_THRESHOLD) {
            Serial.print("Battery low, Pulsing orange: ");
            Serial.println(batteryLevel);
            while (readDataBool && batteryLevel <= LOW_BATT_THRESHOLD) {
                setLedColor(255, 165, 0); vTaskDelay(pdMS_TO_TICKS(750));
                if (!readDataBool) break;
                setLedColor(0, 0, 0); vTaskDelay(pdMS_TO_TICKS(750));
                if (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                    batteryLevel = lipo.getSOC();
                    xSemaphoreGive(i2c_mutex);
                }
            }
            if(readDataBool) setLedColor(0, 255, 0); // Restore green if still running
        }
    }
}
