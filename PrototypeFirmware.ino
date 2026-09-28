// Drone Health Monitoring System
// Arduino UNO Q / STM32U585
// Reads drone telemetry, calculates health, and sends CSV or JSON data.
// Keep analog inputs at or below 3.3 V.

#include <Arduino.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>


// Configuration

namespace Config
{
    // Pin assignments

    constexpr int BATTERY_PIN     = A2;
    constexpr int CURRENT_PIN     = A3;
    constexpr int TEMPERATURE_PIN = A0;
    constexpr int RPM_PIN         = 7;


    // ADC settings

    constexpr uint8_t ADC_BITS = 14;

    constexpr float ADC_REFERENCE_VOLTAGE = 3.3f;

    constexpr float ADC_MAX_VALUE =
        (1UL << ADC_BITS) - 1;


    // Battery voltage divider; change resistor values to match your hardware.

    constexpr float BATTERY_R_TOP_OHMS =
        100000.0f;

    constexpr float BATTERY_R_BOTTOM_OHMS =
        15000.0f;

    constexpr float BATTERY_CALIBRATION =
        1.000f;


    // Current sensor settings; change values to match your sensor.

    constexpr float CURRENT_ZERO_V =
        1.650f;

    constexpr float CURRENT_SENSITIVITY_V_PER_A =
        0.066f;

    constexpr float CURRENT_CALIBRATION =
        1.000f;


    // TMP36-style temperature sensor conversion.

    constexpr float TEMP_ZERO_V =
        0.500f;

    constexpr float TEMP_VOLTS_PER_C =
        0.010f;


    // RPM pulses per motor revolution.

    constexpr float RPM_PULSES_PER_REVOLUTION =
        2.0f;


    // Sampling intervals

    constexpr unsigned long SENSOR_SAMPLE_INTERVAL_MS =
        50;

    // 20 Hz sensor acquisition

    constexpr unsigned long TELEMETRY_INTERVAL_MS =
        100;

    // 10 Hz telemetry output

    constexpr unsigned long RPM_WINDOW_MS =
        500;


    // Sensor filtering

    constexpr float FILTER_ALPHA =
        0.25f;

    constexpr uint8_t ADC_AVERAGE_SAMPLES =
        8;


    // 4S battery voltage limits

    constexpr float BATTERY_FULL_V =
        16.80f;

    constexpr float BATTERY_WARNING_V =
        14.40f;

    constexpr float BATTERY_CRITICAL_V =
        13.20f;


    // Current limits

    constexpr float CURRENT_WARNING_A =
        25.0f;

    constexpr float CURRENT_CRITICAL_A =
        40.0f;


    // Motor temperature limits

    constexpr float TEMPERATURE_WARNING_C =
        70.0f;

    constexpr float TEMPERATURE_CRITICAL_C =
        90.0f;


    // RPM and stall limits

    constexpr float RPM_WARNING =
        12000.0f;

    constexpr float RPM_CRITICAL =
        15000.0f;

    constexpr float STALL_CURRENT_A =
        12.0f;

    constexpr float STALL_RPM =
        800.0f;


    // Start in simulation mode when sensors are not connected.

    constexpr bool DEFAULT_SIMULATION_MODE =
        true;
}


// States and output modes

enum class ComponentState
{
    OK,
    WARNING,
    FAIL
};


enum class SystemState
{
    HEALTHY,
    WARNING,
    CRITICAL
};


enum class OutputFormat
{
    CSV,
    JSON
};


enum class SimulatedFault
{
    NONE,
    BATTERY,
    CURRENT,
    TEMPERATURE,
    MOTOR
};


// Fault flags

enum FaultFlags : uint16_t
{
    FAULT_NONE          = 0,
    FAULT_LOW_BATTERY   = 1 << 0,
    FAULT_OVER_CURRENT  = 1 << 1,
    FAULT_OVER_TEMP     = 1 << 2,
    FAULT_MOTOR_STALL   = 1 << 3,
    FAULT_OVER_SPEED    = 1 << 4,
    FAULT_SENSOR_RANGE  = 1 << 5
};


// Telemetry data

struct Telemetry
{
    unsigned long timestampMs;

    float batteryVoltage;
    float current;
    float temperature;
    float rpm;

    int healthScore;

    ComponentState batteryState;
    ComponentState motorState;
    SystemState systemState;

    uint16_t faults;
};


// Global state

Telemetry telemetry;

bool simulationMode =
    Config::DEFAULT_SIMULATION_MODE;

OutputFormat outputFormat =
    OutputFormat::CSV;

SimulatedFault simulatedFault =
    SimulatedFault::NONE;


// -1 means automatic health calculation.

int healthScoreOverride =
    -1;


// Current sensor zero point.

float currentZeroVoltage =
    Config::CURRENT_ZERO_V;


// Timing

unsigned long previousSensorSample =
    0;

unsigned long previousTelemetryOutput =
    0;

unsigned long rpmWindowStart =
    0;


// RPM measurement

volatile uint32_t rpmPulseCounter =
    0;

float measuredRPM =
    0.0f;


// Sensor filters

bool filterInitialized =
    false;

float filteredBattery =
    0.0f;

float filteredCurrent =
    0.0f;

float filteredTemperature =
    0.0f;


// Serial command buffer

constexpr size_t COMMAND_BUFFER_SIZE =
    96;

char commandBuffer[COMMAND_BUFFER_SIZE];

size_t commandLength =
    0;


// Utility functions

float clampFloat(
    float value,
    float minimum,
    float maximum
)
{
    if (value < minimum)
        return minimum;

    if (value > maximum)
        return maximum;

    return value;
}


// Read ADC voltage

float readADCVoltage(
    int pin
)
{
    uint32_t total =
        0;

    for (
        uint8_t i = 0;
        i < Config::ADC_AVERAGE_SAMPLES;
        i++
    )
    {
        total +=
            analogRead(pin);
    }

    const float averageADC =
        static_cast<float>(total)
        /
        Config::ADC_AVERAGE_SAMPLES;


    return
        (
            averageADC
            /
            Config::ADC_MAX_VALUE
        )
        *
        Config::ADC_REFERENCE_VOLTAGE;
}


// Read battery voltage

float readBatteryVoltage()
{
    const float adcVoltage =
        readADCVoltage(
            Config::BATTERY_PIN
        );


    const float dividerRatio =

        (
            Config::BATTERY_R_TOP_OHMS
            +
            Config::BATTERY_R_BOTTOM_OHMS
        )

        /

        Config::BATTERY_R_BOTTOM_OHMS;


    return
        adcVoltage
        *
        dividerRatio
        *
        Config::BATTERY_CALIBRATION;
}


// Read current

float readCurrent()
{
    const float sensorVoltage =
        readADCVoltage(
            Config::CURRENT_PIN
        );


    float current =

        (
            sensorVoltage
            -
            currentZeroVoltage
        )

        /

        Config::CURRENT_SENSITIVITY_V_PER_A;


    current *=
        Config::CURRENT_CALIBRATION;


    // Remove tiny zero-current noise.

    if (
        fabsf(current) < 0.10f
    )
    {
        current =
            0.0f;
    }


    return current;
}


// Read temperature

float readTemperature()
{
    const float sensorVoltage =
        readADCVoltage(
            Config::TEMPERATURE_PIN
        );


    return

        (
            sensorVoltage
            -
            Config::TEMP_ZERO_V
        )

        /

        Config::TEMP_VOLTS_PER_C;
}


// Count RPM pulses

void rpmPulseISR()
{
    rpmPulseCounter++;
}


// Calculate RPM

void updateRPM(
    unsigned long now
)
{
    const unsigned long elapsed =

        now
        -
        rpmWindowStart;


    if (
        elapsed
        <
        Config::RPM_WINDOW_MS
    )
    {
        return;
    }


    noInterrupts();

    const uint32_t pulses =
        rpmPulseCounter;

    rpmPulseCounter =
        0;

    interrupts();


    measuredRPM =

        (
            static_cast<float>(pulses)
            *
            60000.0f
        )

        /

        (
            static_cast<float>(elapsed)
            *
            Config::RPM_PULSES_PER_REVOLUTION
        );


    rpmWindowStart =
        now;
}


// Low-pass filter

float lowPassFilter(
    float previousValue,
    float newValue
)
{
    return

        Config::FILTER_ALPHA
        *
        newValue

        +

        (
            1.0f
            -
            Config::FILTER_ALPHA
        )
        *
        previousValue;
}


// Read real sensors

void acquireRealSensors(
    unsigned long now
)
{
    updateRPM(now);


    const float battery =
        readBatteryVoltage();

    const float current =
        readCurrent();

    const float temperature =
        readTemperature();


    if (
        !filterInitialized
    )
    {
        filteredBattery =
            battery;

        filteredCurrent =
            current;

        filteredTemperature =
            temperature;

        filterInitialized =
            true;
    }

    else
    {
        filteredBattery =
            lowPassFilter(
                filteredBattery,
                battery
            );

        filteredCurrent =
            lowPassFilter(
                filteredCurrent,
                current
            );

        filteredTemperature =
            lowPassFilter(
                filteredTemperature,
                temperature
            );
    }


    telemetry.timestampMs =
        now;

    telemetry.batteryVoltage =
        filteredBattery;

    telemetry.current =
        filteredCurrent;

    telemetry.temperature =
        filteredTemperature;

    telemetry.rpm =
        measuredRPM;
}


// Generate simulated sensor data

void acquireSimulatedSensors(
    unsigned long now
)
{
    const float t =
        static_cast<float>(now)
        /
        1000.0f;


    telemetry.timestampMs =
        now;


    telemetry.batteryVoltage =

        15.8f
        +
        0.08f
        *
        sinf(t * 0.15f);


    telemetry.current =

        8.4f
        +
        1.8f
        *
        sinf(t * 0.70f);


    telemetry.temperature =

        46.0f
        +
        2.5f
        *
        sinf(t * 0.10f);


    telemetry.rpm =

        7350.0f
        +
        400.0f
        *
        sinf(t * 0.60f);


    // Apply simulated faults.

    switch (
        simulatedFault
    )
    {
        case SimulatedFault::BATTERY:

            telemetry.batteryVoltage =
                12.8f;

            break;


        case SimulatedFault::CURRENT:

            telemetry.current =
                43.0f;

            break;


        case SimulatedFault::TEMPERATURE:

            telemetry.temperature =
                95.0f;

            break;


        case SimulatedFault::MOTOR:

            telemetry.current =
                18.0f;

            telemetry.temperature =
                78.0f;

            telemetry.rpm =
                350.0f;

            break;


        case SimulatedFault::NONE:

        default:

            break;
    }
}


// Check sensor ranges

bool sensorValuesValid()
{
    if (
        telemetry.batteryVoltage < 0.0f
        ||
        telemetry.batteryVoltage > 30.0f
    )
    {
        return false;
    }


    if (
        telemetry.current < -100.0f
        ||
        telemetry.current > 100.0f
    )
    {
        return false;
    }


    if (
        telemetry.temperature < -40.0f
        ||
        telemetry.temperature > 150.0f
    )
    {
        return false;
    }


    if (
        telemetry.rpm < 0.0f
        ||
        telemetry.rpm > 100000.0f
    )
    {
        return false;
    }


    return true;
}


// Calculate battery health

float calculateBatteryScore()
{
    if (
        telemetry.batteryVoltage
        <=
        Config::BATTERY_CRITICAL_V
    )
    {
        return 0.0f;
    }


    if (
        telemetry.batteryVoltage
        >=
        Config::BATTERY_FULL_V
    )
    {
        return 100.0f;
    }


    return

        100.0f
        *
        (
            telemetry.batteryVoltage
            -
            Config::BATTERY_CRITICAL_V
        )

        /

        (
            Config::BATTERY_FULL_V
            -
            Config::BATTERY_CRITICAL_V
        );
}


// Calculate score for upper limits

float calculateUpperLimitScore(
    float value,
    float warningLevel,
    float criticalLevel
)
{
    value =
        fabsf(value);


    if (
        value
        <=
        warningLevel
    )
    {
        return 100.0f;
    }


    if (
        value
        >=
        criticalLevel
    )
    {
        return 0.0f;
    }


    return

        100.0f
        *
        (
            criticalLevel
            -
            value
        )

        /

        (
            criticalLevel
            -
            warningLevel
        );
}


// Calculate RPM health

float calculateRPMScore()
{
    const bool motorStalled =

        fabsf(telemetry.current)
            >=
        Config::STALL_CURRENT_A

        &&

        telemetry.rpm
            <
        Config::STALL_RPM;


    if (
        motorStalled
    )
    {
        return 0.0f;
    }


    return
        calculateUpperLimitScore(
            telemetry.rpm,
            Config::RPM_WARNING,
            Config::RPM_CRITICAL
        );
}


// Calculate health score and fault states

void evaluateSystemHealth()
{
    telemetry.faults =
        FAULT_NONE;


    // Check sensor values.

    if (
        !sensorValuesValid()
    )
    {
        telemetry.faults |=
            FAULT_SENSOR_RANGE;
    }


    // Check battery state.

    if (
        telemetry.batteryVoltage
        <=
        Config::BATTERY_CRITICAL_V
    )
    {
        telemetry.batteryState =
            ComponentState::FAIL;

        telemetry.faults |=
            FAULT_LOW_BATTERY;
    }

    else if (
        telemetry.batteryVoltage
        <=
        Config::BATTERY_WARNING_V
    )
    {
        telemetry.batteryState =
            ComponentState::WARNING;

        telemetry.faults |=
            FAULT_LOW_BATTERY;
    }

    else
    {
        telemetry.batteryState =
            ComponentState::OK;
    }


    // Check motor state.

    bool motorWarning =
        false;

    bool motorFailure =
        false;


    if (
        telemetry.temperature
        >=
        Config::TEMPERATURE_WARNING_C
    )
    {
        telemetry.faults |=
            FAULT_OVER_TEMP;

        motorWarning =
            true;
    }


    if (
        telemetry.temperature
        >=
        Config::TEMPERATURE_CRITICAL_C
    )
    {
        motorFailure =
            true;
    }


    if (
        fabsf(telemetry.current)
        >=
        Config::CURRENT_WARNING_A
    )
    {
        telemetry.faults |=
            FAULT_OVER_CURRENT;

        motorWarning =
            true;
    }


    if (
        fabsf(telemetry.current)
        >=
        Config::CURRENT_CRITICAL_A
    )
    {
        motorFailure =
            true;
    }


    const bool stalled =

        fabsf(telemetry.current)
            >=
        Config::STALL_CURRENT_A

        &&

        telemetry.rpm
            <
        Config::STALL_RPM;


    if (
        stalled
    )
    {
        telemetry.faults |=
            FAULT_MOTOR_STALL;

        motorFailure =
            true;
    }


    if (
        telemetry.rpm
        >=
        Config::RPM_WARNING
    )
    {
        telemetry.faults |=
            FAULT_OVER_SPEED;

        motorWarning =
            true;
    }


    if (
        telemetry.rpm
        >=
        Config::RPM_CRITICAL
    )
    {
        motorFailure =
            true;
    }


    if (
        motorFailure
    )
    {
        telemetry.motorState =
            ComponentState::FAIL;
    }

    else if (
        motorWarning
    )
    {
        telemetry.motorState =
            ComponentState::WARNING;
    }

    else
    {
        telemetry.motorState =
            ComponentState::OK;
    }


    // Calculate health scores.

    const float batteryScore =
        calculateBatteryScore();


    const float currentScore =
        calculateUpperLimitScore(
            telemetry.current,
            Config::CURRENT_WARNING_A,
            Config::CURRENT_CRITICAL_A
        );


    const float temperatureScore =
        calculateUpperLimitScore(
            telemetry.temperature,
            Config::TEMPERATURE_WARNING_C,
            Config::TEMPERATURE_CRITICAL_C
        );


    const float rpmScore =
        calculateRPMScore();


    // Weighted health score.

    float health =

        0.35f * batteryScore

        +

        0.20f * currentScore

        +

        0.25f * temperatureScore

        +

        0.20f * rpmScore;


    health =
        clampFloat(
            health,
            0.0f,
            100.0f
        );


    // Match score to component states.

    if (
        telemetry.batteryState
            ==
        ComponentState::FAIL

        ||

        telemetry.motorState
            ==
        ComponentState::FAIL
    )
    {
        health =
            min(
                health,
                49.0f
            );
    }

    else if (
        telemetry.batteryState
            ==
        ComponentState::WARNING

        ||

        telemetry.motorState
            ==
        ComponentState::WARNING
    )
    {
        health =
            min(
                health,
                79.0f
            );
    }


    // Use manual health score when set.

    if (
        healthScoreOverride
        >=
        0
    )
    {
        health =
            healthScoreOverride;
    }


    telemetry.healthScore =
        static_cast<int>(
            roundf(health)
        );


    // Set overall system state.

    if (
        telemetry.healthScore
        < 50
    )
    {
        telemetry.systemState =
            SystemState::CRITICAL;
    }

    else if (
        telemetry.healthScore
        < 80
    )
    {
        telemetry.systemState =
            SystemState::WARNING;
    }

    else
    {
        telemetry.systemState =
            SystemState::HEALTHY;
    }
}


// Convert states to text

const char* componentStateText(
    ComponentState state
)
{
    switch (state)
    {
        case ComponentState::OK:
            return "OK";

        case ComponentState::WARNING:
            return "WARNING";

        case ComponentState::FAIL:
            return "FAIL";

        default:
            return "UNKNOWN";
    }
}


const char* systemStateText(
    SystemState state
)
{
    switch (state)
    {
        case SystemState::HEALTHY:
            return "HEALTHY";

        case SystemState::WARNING:
            return "WARNING";

        case SystemState::CRITICAL:
            return "CRITICAL";

        default:
            return "UNKNOWN";
    }
}


// Print fault names

void printFaults(
    Print& output,
    uint16_t faults
)
{
    if (
        faults == FAULT_NONE
    )
    {
        output.print("NONE");

        return;
    }


    bool first =
        true;


    auto printFault =
        [&](const char* text)
        {
            if (!first)
            {
                output.print("|");
            }

            output.print(text);

            first =
                false;
        };


    if (
        faults
        &
        FAULT_LOW_BATTERY
    )
    {
        printFault(
            "LOW_BATTERY"
        );
    }


    if (
        faults
        &
        FAULT_OVER_CURRENT
    )
    {
        printFault(
            "OVER_CURRENT"
        );
    }


    if (
        faults
        &
        FAULT_OVER_TEMP
    )
    {
        printFault(
            "OVER_TEMP"
        );
    }


    if (
        faults
        &
        FAULT_MOTOR_STALL
    )
    {
        printFault(
            "MOTOR_STALL"
        );
    }


    if (
        faults
        &
        FAULT_OVER_SPEED
    )
    {
        printFault(
            "OVER_SPEED"
        );
    }


    if (
        faults
        &
        FAULT_SENSOR_RANGE
    )
    {
        printFault(
            "SENSOR_RANGE"
        );
    }
}


// CSV output

void printCSVHeader()
{
    Serial.println(
        "time_ms,"
        "battery_v,"
        "current_a,"
        "temperature_c,"
        "rpm,"
        "health_score,"
        "battery_status,"
        "motor_status,"
        "system_status,"
        "faults"
    );
}


void printCSV()
{
    Serial.print(
        telemetry.timestampMs
    );

    Serial.print(",");


    Serial.print(
        telemetry.batteryVoltage,
        2
    );

    Serial.print(",");


    Serial.print(
        telemetry.current,
        2
    );

    Serial.print(",");


    Serial.print(
        telemetry.temperature,
        1
    );

    Serial.print(",");


    Serial.print(
        telemetry.rpm,
        0
    );

    Serial.print(",");


    Serial.print(
        telemetry.healthScore
    );

    Serial.print(",");


    Serial.print(
        componentStateText(
            telemetry.batteryState
        )
    );

    Serial.print(",");


    Serial.print(
        componentStateText(
            telemetry.motorState
        )
    );

    Serial.print(",");


    Serial.print(
        systemStateText(
            telemetry.systemState
        )
    );

    Serial.print(",");


    printFaults(
        Serial,
        telemetry.faults
    );


    Serial.println();
}


// JSON output

void printJSON()
{
    Serial.print(
        "{\"time_ms\":"
    );

    Serial.print(
        telemetry.timestampMs
    );


    Serial.print(
        ",\"battery_v\":"
    );

    Serial.print(
        telemetry.batteryVoltage,
        2
    );


    Serial.print(
        ",\"current_a\":"
    );

    Serial.print(
        telemetry.current,
        2
    );


    Serial.print(
        ",\"temperature_c\":"
    );

    Serial.print(
        telemetry.temperature,
        1
    );


    Serial.print(
        ",\"rpm\":"
    );

    Serial.print(
        telemetry.rpm,
        0
    );


    Serial.print(
        ",\"health_score\":"
    );

    Serial.print(
        telemetry.healthScore
    );


    Serial.print(
        ",\"battery_status\":\""
    );

    Serial.print(
        componentStateText(
            telemetry.batteryState
        )
    );

    Serial.print(
        "\""
    );


    Serial.print(
        ",\"motor_status\":\""
    );

    Serial.print(
        componentStateText(
            telemetry.motorState
        )
    );

    Serial.print(
        "\""
    );


    Serial.print(
        ",\"system_status\":\""
    );

    Serial.print(
        systemStateText(
            telemetry.systemState
        )
    );

    Serial.print(
        "\""
    );


    Serial.print(
        ",\"faults\":\""
    );

    printFaults(
        Serial,
        telemetry.faults
    );

    Serial.println(
        "\"}"
    );
}


// Send telemetry

void outputTelemetry()
{
    if (
        outputFormat
        ==
        OutputFormat::CSV
    )
    {
        printCSV();
    }

    else
    {
        printJSON();
    }
}


// Calibrate current sensor zero

void calibrateCurrentSensor()
{
    Serial.println(
        "# Current zero calibration starting."
    );

    Serial.println(
        "# Ensure motor/load current is zero."
    );


    const int sampleCount =
        100;


    float totalVoltage =
        0.0f;


    for (
        int i = 0;
        i < sampleCount;
        i++
    )
    {
        totalVoltage +=
            readADCVoltage(
                Config::CURRENT_PIN
            );

        delay(5);
    }


    currentZeroVoltage =

        totalVoltage
        /
        sampleCount;


    Serial.print(
        "# New current zero voltage: "
    );

    Serial.println(
        currentZeroVoltage,
        4
    );
}


// Print command list

void printHelp()
{
    Serial.println(
        "# ================================"
    );

    Serial.println(
        "# DRONE HEALTH COMMANDS"
    );

    Serial.println(
        "# ================================"
    );

    Serial.println(
        "# HELP"
    );

    Serial.println(
        "# STATUS"
    );

    Serial.println(
        "# SIM ON"
    );

    Serial.println(
        "# SIM OFF"
    );

    Serial.println(
        "# FORMAT CSV"
    );

    Serial.println(
        "# FORMAT JSON"
    );

    Serial.println(
        "# HEADER"
    );

    Serial.println(
        "# HEALTH AUTO"
    );

    Serial.println(
        "# HEALTH <0-100>"
    );

    Serial.println(
        "# FAULT NONE"
    );

    Serial.println(
        "# FAULT BATTERY"
    );

    Serial.println(
        "# FAULT CURRENT"
    );

    Serial.println(
        "# FAULT TEMPERATURE"
    );

    Serial.println(
        "# FAULT MOTOR"
    );

    Serial.println(
        "# CAL CURRENT"
    );

    Serial.println(
        "# ================================"
    );
}


// Print firmware status

void printFirmwareStatus()
{
    Serial.println(
        "# ================================"
    );

    Serial.println(
        "# Drone Health Firmware"
    );


    Serial.print(
        "# Simulation: "
    );

    Serial.println(
        simulationMode
        ?
        "ON"
        :
        "OFF"
    );


    Serial.print(
        "# Format: "
    );

    Serial.println(
        outputFormat
            ==
        OutputFormat::CSV

        ?

        "CSV"

        :

        "JSON"
    );


    Serial.print(
        "# Health override: "
    );


    if (
        healthScoreOverride < 0
    )
    {
        Serial.println(
            "AUTO"
        );
    }

    else
    {
        Serial.println(
            healthScoreOverride
        );
    }


    Serial.println(
        "# ================================"
    );
}


// Handle serial commands

void handleCommand(
    char* command
)
{
    // Convert command to uppercase.

    for (
        size_t i = 0;
        command[i] != '\0';
        i++
    )
    {
        command[i] =
            static_cast<char>(
                toupper(
                    command[i]
                )
            );
    }


    // HELP command

    if (
        strcmp(
            command,
            "HELP"
        )
        ==
        0
    )
    {
        printHelp();

        return;
    }


    // STATUS command

    if (
        strcmp(
            command,
            "STATUS"
        )
        ==
        0
    )
    {
        printFirmwareStatus();

        return;
    }


    // Simulation commands

    if (
        strcmp(
            command,
            "SIM ON"
        )
        ==
        0
    )
    {
        simulationMode =
            true;

        Serial.println(
            "# Simulation enabled."
        );

        return;
    }


    if (
        strcmp(
            command,
            "SIM OFF"
        )
        ==
        0
    )
    {
        simulationMode =
            false;

        filterInitialized =
            false;

        Serial.println(
            "# Real sensors enabled."
        );

        return;
    }


    // Output format commands

    if (
        strcmp(
            command,
            "FORMAT CSV"
        )
        ==
        0
    )
    {
        outputFormat =
            OutputFormat::CSV;

        Serial.println(
            "# CSV output enabled."
        );

        printCSVHeader();

        return;
    }


    if (
        strcmp(
            command,
            "FORMAT JSON"
        )
        ==
        0
    )
    {
        outputFormat =
            OutputFormat::JSON;

        Serial.println(
            "# JSON output enabled."
        );

        return;
    }


    // Print CSV header

    if (
        strcmp(
            command,
            "HEADER"
        )
        ==
        0
    )
    {
        printCSVHeader();

        return;
    }


    // Enable automatic health score

    if (
        strcmp(
            command,
            "HEALTH AUTO"
        )
        ==
        0
    )
    {
        healthScoreOverride =
            -1;

        Serial.println(
            "# Automatic health calculation enabled."
        );

        return;
    }


    // Set manual health score

    if (
        strncmp(
            command,
            "HEALTH ",
            7
        )
        ==
        0
    )
    {
        const int value =
            atoi(
                command + 7
            );


        if (
            value >= 0
            &&
            value <= 100
        )
        {
            healthScoreOverride =
                value;


            Serial.print(
                "# Health score override = "
            );

            Serial.println(
                healthScoreOverride
            );
        }

        else
        {
            Serial.println(
                "# ERROR: Health must be 0-100."
            );
        }


        return;
    }


    // Simulated fault commands

    if (
        strcmp(
            command,
            "FAULT NONE"
        )
        ==
        0
    )
    {
        simulatedFault =
            SimulatedFault::NONE;

        Serial.println(
            "# Simulated fault cleared."
        );

        return;
    }


    if (
        strcmp(
            command,
            "FAULT BATTERY"
        )
        ==
        0
    )
    {
        simulatedFault =
            SimulatedFault::BATTERY;

        Serial.println(
            "# Battery fault injected."
        );

        return;
    }


    if (
        strcmp(
            command,
            "FAULT CURRENT"
        )
        ==
        0
    )
    {
        simulatedFault =
            SimulatedFault::CURRENT;

        Serial.println(
            "# Current fault injected."
        );

        return;
    }


    if (
        strcmp(
            command,
            "FAULT TEMPERATURE"
        )
        ==
        0
    )
    {
        simulatedFault =
            SimulatedFault::TEMPERATURE;

        Serial.println(
            "# Temperature fault injected."
        );

        return;
    }


    if (
        strcmp(
            command,
            "FAULT MOTOR"
        )
        ==
        0
    )
    {
        simulatedFault =
            SimulatedFault::MOTOR;

        Serial.println(
            "# Motor fault injected."
        );

        return;
    }


    // Calibrate current sensor

    if (
        strcmp(
            command,
            "CAL CURRENT"
        )
        ==
        0
    )
    {
        calibrateCurrentSensor();

        return;
    }


    Serial.println(
        "# ERROR: Unknown command. Type HELP."
    );
}


// Read serial commands

void processSerialCommands()
{
    while (
        Serial.available() > 0
    )
    {
        const char character =
            Serial.read();


        if (
            character == '\r'
        )
        {
            continue;
        }


        if (
            character == '\n'
        )
        {
            if (
                commandLength > 0
            )
            {
                commandBuffer[
                    commandLength
                ] = '\0';


                handleCommand(
                    commandBuffer
                );


                commandLength =
                    0;
            }


            continue;
        }


        if (
            commandLength
            <
            COMMAND_BUFFER_SIZE - 1
        )
        {
            commandBuffer[
                commandLength++
            ] =
                character;
        }
    }
}


// Setup

void setup()
{
    Serial.begin(
        115200
    );


    analogReadResolution(
        Config::ADC_BITS
    );


    pinMode(
        Config::BATTERY_PIN,
        INPUT
    );


    pinMode(
        Config::CURRENT_PIN,
        INPUT
    );


    pinMode(
        Config::TEMPERATURE_PIN,
        INPUT
    );


    pinMode(
        Config::RPM_PIN,
        INPUT_PULLUP
    );


    attachInterrupt(
        digitalPinToInterrupt(
            Config::RPM_PIN
        ),
        rpmPulseISR,
        RISING
    );


    rpmWindowStart =
        millis();


    Serial.println();
    Serial.println(
        "# Drone Health Monitoring System"
    );

    Serial.println(
        "# Arduino UNO Q / STM32U585"
    );

    Serial.println(
        "# Firmware started."
    );


    Serial.print(
        "# Simulation mode: "
    );

    Serial.println(
        simulationMode
        ?
        "ON"
        :
        "OFF"
    );


    Serial.println(
        "# Type HELP for commands."
    );

    Serial.println();


    if (
        outputFormat
        ==
        OutputFormat::CSV
    )
    {
        printCSVHeader();
    }
}


// Main loop

void loop()
{
    processSerialCommands();


    const unsigned long now =
        millis();


    // Read sensors

    if (
        now
        -
        previousSensorSample
        >=
        Config::SENSOR_SAMPLE_INTERVAL_MS
    )
    {
        previousSensorSample =
            now;


        if (
            simulationMode
        )
        {
            acquireSimulatedSensors(
                now
            );
        }

        else
        {
            acquireRealSensors(
                now
            );
        }


        evaluateSystemHealth();
    }


    // Send telemetry

    if (
        now
        -
        previousTelemetryOutput
        >=
        Config::TELEMETRY_INTERVAL_MS
    )
    {
        previousTelemetryOutput =
            now;


        outputTelemetry();
    }
}
