/*
 * GPS.c
 *
 *  Created on: Oct 29, 2025
 *      Author: kemazhu
 */

#include "stm32f4xx_hal.h"
#include "gps.h"
#include "main.h"
#include "string.h"
#include "stdlib.h"

// ========================
// Configuration
// ========================
#define GPS_BUFFER_SIZE         256
#define GPS_UART                huart2  // Doit correspondre à main.c

// ========================
// Variables privées
// ========================
static uint8_t dma_buffer[GPS_BUFFER_SIZE];
static volatile uint16_t buffer_head = 0; // Dernière position traitée
static GPS_Data_t latest_data = {0};
static volatile bool new_data_ready = false;
static UART_HandleTypeDef *gps_huart = NULL;

// ========================
// Fonctions internes
// ========================
static double parse_lat_lon(const char *str);
static bool parse_gpgga_line(const char *line, GPS_Data_t *out);
static uint16_t get_dma_tail(void);

// ========================
// Implémentation
// ========================

void GPS_Init(UART_HandleTypeDef *huart) {
    gps_huart = huart;
    buffer_head = 0;
    new_data_ready = false;
    memset(&latest_data, 0, sizeof(latest_data));

    // Démarrer la réception DMA en mode circulaire
    HAL_UART_Receive_DMA(gps_huart, dma_buffer, GPS_BUFFER_SIZE);
}

static uint16_t get_dma_tail(void) {
    // NDTR = nombre d'éléments restants → position courante = BUFFER_SIZE - NDTR
    return GPS_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(gps_huart->hdmarx);
}

void GPS_Update(void) {
    static uint16_t last_tail = 0;
    uint16_t tail = get_dma_tail();

    if (tail != last_tail) {
        static char line[GPS_BUFFER_SIZE];
        static uint16_t line_pos = 0;

        uint16_t start = last_tail;
        uint16_t end = (tail > start) ? tail : tail + GPS_BUFFER_SIZE;

        for (uint16_t i = start; i < end; i++) {
            uint8_t c = dma_buffer[i % GPS_BUFFER_SIZE];

            if (c == '\n') {
                if (line_pos > 0 && line[0] == '$') {
                    if (strncmp(line, "$GPGGA", 6) == 0) {
                        if (parse_gpgga_line(line, &latest_data)) {
                            new_data_ready = true;
                        }
                    }
                }
                line_pos = 0;
            } else if (c >= 32 && c <= 126 && line_pos < sizeof(line) - 1) {
                line[line_pos++] = (char)c;
            }
        }

        last_tail = tail % GPS_BUFFER_SIZE;
    }
}

bool GPS_GetLatestData(GPS_Data_t *data) {
    if (new_data_ready && data != NULL) {
        *data = latest_data;
        new_data_ready = false;
        return true;
    }
    return false;
}

// ========================
// Parsing NMEA
// ========================

static double parse_lat_lon(const char *str) {
    if (str == NULL || str[0] == '\0') return 0.0;
    double val = atof(str);
    int degrees = (int)(val / 100);
    double minutes = val - (degrees * 100);
    return degrees + minutes / 60.0;
}

static bool parse_gpgga_line(const char *line, GPS_Data_t *out) {
    char *tokens[15];
    char temp[GPS_BUFFER_SIZE];
    strncpy(temp, line, sizeof(temp) - 1);
    temp[sizeof(temp) - 1] = '\0';

    char *token = strtok(temp, ",");
    int count = 0;
    while (token != NULL && count < 15) {
        tokens[count++] = token;
        token = strtok(NULL, ",");
    }

    if (count < 10) return false;

    uint8_t fix_quality = (tokens[6][0] == '\0') ? 0 : (uint8_t)atoi(tokens[6]);
    if (fix_quality == 0) {
        out->valid = false;
        return false;
    }

    out->valid = true;
    out->utc_time = (tokens[1][0] == '\0') ? 0 : atol(tokens[1]);
    out->fix_quality = fix_quality;
    out->num_satellites = (tokens[7][0] == '\0') ? 0 : (uint8_t)atoi(tokens[7]);
    out->altitude = (tokens[9][0] == '\0') ? 0.0 : atof(tokens[9]);

    double lat = parse_lat_lon(tokens[2]);
    if (tokens[3][0] == 'S') lat = -lat;
    out->latitude = lat;

    double lon = parse_lat_lon(tokens[4]);
    if (tokens[5][0] == 'W') lon = -lon;
    out->longitude = lon;

    return true;
}
