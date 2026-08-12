/**
 * @file    GPS.c
 * @brief   L86-M33 GPS NMEA parser and MTK command driver for STM32F4xx.
 *
 * @details Implements a byte-by-byte NMEA sentence accumulator (ISR-safe)
 *          and a main-loop parser for GPRMC and GPGGA sentences.
 *          MTK commands are framed and checksummed automatically.
 *
 * @date    April 16, 2026
 * @author  César Pérez
 * @version 3.0.0
 */

#include "GPS.h"
#include "Logger.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* ========================  EXTERNAL HAL HANDLES  ========================== */

extern UART_HandleTypeDef huart1;

/* ========================  STATIC HELPERS  ================================ */

static uint8_t nmea_checksum(const char *s, uint16_t len)
{
    uint8_t cs = 0U;
    for (uint16_t i = 0U; i < len; i++) {
        cs ^= (uint8_t)s[i];
    }
    return cs;
}

static uint8_t hex_to_val(char c)
{
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10U);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10U);
    return 0U;
}

static const char *next_field(const char *p)
{
    if (p == NULL) return NULL;
    while (*p != '\0') {
        if (*p == ',') return p + 1;
        p++;
    }
    return NULL;
}

static double parse_double(const char *p)
{
    if (p == NULL || *p == ',' || *p == '\0') return 0.0;
    return atof(p);
}

static int parse_int(const char *p)
{
    if (p == NULL || *p == ',' || *p == '\0') return 0;
    return atoi(p);
}

static double nmea_to_degrees(double raw, char direction)
{
    int deg = (int)(raw / 100.0);
    double min = raw - (deg * 100.0);
    double result = (double)deg + (min / 60.0);

    if (direction == 'S' || direction == 'W') {
        result = -result;
    }
    return result;
}

/**
 * @brief  Returns the UTC offset in hours for a given longitude using
 *         standard 15-degree timezone bands with political corrections.
 * @param  lon  Longitude in decimal degrees (+ East / - West).
 * @retval UTC offset in hours (e.g. -6 for CST, +1 for CET).
 */
int8_t Gps_GetTimezoneOffset(double lon)
{
    typedef struct { int16_t lon_min; int16_t lon_max; int8_t offset; } TzEntry_t;

    static const TzEntry_t tz_table[] = {
        /* West — Americas */
        { -180, -172,  -12 },   /* Baker / Howland Islands                    */
        { -172, -157,  -11 },   /* Samoa, Niue                                */
        { -157, -142,  -10 },   /* Hawaii, Cook Islands                       */
        { -142, -127,   -9 },   /* Alaska                                     */
        { -127, -112,   -8 },   /* PST — California, BC, Tijuana              */
        { -112, -104,   -7 },   /* MST — Sonora (-110), Arizona, Colorado     */
        { -104,  -82,   -6 },   /* CST — CDMX (-99), Monterrey, Chicago       */
        {  -82,  -67,   -5 },   /* EST — Nueva York, Bogotá (-74), Lima (-77) */
        {  -67,  -60,   -4 },   /* AST — Venezuela (-67), Trinidad            */
        {  -60,  -37,   -3 },   /* BRT — Brasil, Argentina (-58), Uruguay     */
        {  -37,  -22,   -2 },   /* Fernando de Noronha                        */
        {  -22,   -7,   -1 },   /* Azores, Cabo Verde                         */
        /* Prime meridian / Europa / África */
        {   -7,    8,    0 },   /* GMT — UK, Portugal, Ghana         */
        {    8,   23,    1 },   /* CET — Europa central, África Occ  */
        {   23,   38,    2 },   /* EET — Europa del Este, Egipto     */
        {   38,   53,    3 },   /* MSK — Moscú, Nairobi, Arabia      */
        {   53,   60,    4 },   /* GST — Dubai, Georgia              */
        {   60,   68,    5 },   /* PKT — Pakistán, Kazajistán        */
        {   68,   76,    6 },   /* BST — Bangladés, Butan            */
        {   76,   91,    7 },   /* ICT — Bangkok, Jakarta, Vietnam   */
        {   91,  106,    8 },   /* CST — China, Singapur, Perth      */
        {  106,  121,    9 },   /* JST — Japón, Corea                */
        {  121,  136,   10 },   /* AEST — Sydney, Guam               */
        {  136,  151,   11 },   /* SBT — Islas Salomón               */
        {  151,  180,   12 },   /* NZST — Nueva Zelanda, Fiji        */
    };

    for (uint8_t i = 0U; i < (uint8_t)(sizeof(tz_table) / sizeof(tz_table[0])); i++) {
        if (lon >= (double)tz_table[i].lon_min && lon < (double)tz_table[i].lon_max) {
            return tz_table[i].offset;
        }
    }

    return GPS_UTC_OFFSET_H;
}

/**
 * @brief  Applies the auto-detected timezone offset to the parsed UTC date/time.
 * @param  data  Pointer to GpsData_t with UTC values already set.
 */
static void apply_timezone(GpsData_t *data)
{
    int16_t h = (int16_t)data->hour + (int16_t)Gps_GetTimezoneOffset(data->longitude);

    if (h < 0) {
        h += 24;
        /* Roll back one day */
        if (data->day > 1U) {
            data->day--;
        } else {
            /* Roll back month (simplified: no month-length table needed for GPS) */
            if (data->month > 1U) {
                data->month--;
            } else {
                data->month = 12U;
                data->year--;
            }
            /* Approximate days in previous month */
            static const uint8_t days_in_month[] = {
                31,28,31,30,31,30,31,31,30,31,30,31
            };
            data->day = days_in_month[data->month - 1U];
        }
    } else if (h >= 24) {
        h -= 24;
        /* Roll forward one day (simplified) */
        data->day++;
        static const uint8_t days_in_month[] = {
            31,28,31,30,31,30,31,31,30,31,30,31
        };
        if (data->day > days_in_month[data->month - 1U]) {
            data->day = 1U;
            data->month++;
            if (data->month > 12U) {
                data->month = 1U;
                data->year++;
            }
        }
    }

    data->hour = (uint8_t)h;
}

/* ========================  SENTENCE PARSERS  ============================== */

/**
 * @brief  Parses a GPRMC sentence.
 *
 * Fields: time, status, lat, N/S, lon, E/W, speed(knots), course, date
 */
static GpsStatus_e parse_rmc(const char *body, GpsData_t *data)
{
    const char *p = body;

    /* Field 0: Time — hhmmss.ss */
    double raw_time = parse_double(p);
    int time_int = (int)raw_time;

    p = next_field(p);  /* Field 1: Status */
    if (p == NULL) return GPS_ERR_PARSE;
    char status = *p;

    p = next_field(p);  /* Field 2: Latitude */
    if (p == NULL) return GPS_ERR_PARSE;
    double raw_lat = parse_double(p);

    p = next_field(p);  /* Field 3: N/S */
    if (p == NULL) return GPS_ERR_PARSE;
    char ns = *p;

    p = next_field(p);  /* Field 4: Longitude */
    if (p == NULL) return GPS_ERR_PARSE;
    double raw_lon = parse_double(p);

    p = next_field(p);  /* Field 5: E/W */
    if (p == NULL) return GPS_ERR_PARSE;
    char ew = *p;

    p = next_field(p);  /* Field 6: Speed (knots) */
    if (p == NULL) return GPS_ERR_PARSE;
    double knots = parse_double(p);

    p = next_field(p);  /* Field 7: Course */
    if (p == NULL) return GPS_ERR_PARSE;
    double course = parse_double(p);

    p = next_field(p);  /* Field 8: Date — ddmmyy */
    if (p == NULL) return GPS_ERR_PARSE;
    int raw_date = parse_int(p);

    /* Position: only if status Active */
    if (status == 'A') {
        data->latitude  = nmea_to_degrees(raw_lat, ns);
        data->longitude = nmea_to_degrees(raw_lon, ew);
        data->speed_kmh = knots * 1.852;
        data->course    = course;
        data->position_valid = true;
    } else {
        data->position_valid = false;
    }

    /* Time (UTC) */
    if (time_int > 0) {
        data->hour   = (uint8_t)(time_int / 10000);
        data->minute = (uint8_t)((time_int / 100) % 100);
        data->second = (uint8_t)(time_int % 100);
    }

    /* Date */
    if (raw_date > 0) {
        data->day   = (uint8_t)(raw_date / 10000);
        data->month = (uint8_t)((raw_date / 100) % 100);
        data->year  = (uint16_t)(2000U + (raw_date % 100));
        data->datetime_valid = true;
    }

    /* Convert UTC → local time */
    if (data->datetime_valid) {
        apply_timezone(data);
    }

    return GPS_OK;
}

/**
 * @brief  Parses a GPGGA sentence.
 *
 * Fields: time, lat, N/S, lon, E/W, fix, sats, hdop, alt, alt_unit
 */
static GpsStatus_e parse_gga(const char *body, GpsData_t *data)
{
    const char *p = body;

    /* Field 0: Time — skip (already handled in RMC) */
    p = next_field(p);  /* Field 1: Latitude */
    if (p == NULL) return GPS_ERR_PARSE;
    double raw_lat = parse_double(p);

    p = next_field(p);  /* Field 2: N/S */
    if (p == NULL) return GPS_ERR_PARSE;
    char ns = *p;

    p = next_field(p);  /* Field 3: Longitude */
    if (p == NULL) return GPS_ERR_PARSE;
    double raw_lon = parse_double(p);

    p = next_field(p);  /* Field 4: E/W */
    if (p == NULL) return GPS_ERR_PARSE;
    char ew = *p;

    p = next_field(p);  /* Field 5: Fix quality */
    if (p == NULL) return GPS_ERR_PARSE;
    int fix = parse_int(p);

    p = next_field(p);  /* Field 6: Satellites */
    if (p == NULL) return GPS_ERR_PARSE;
    int sats = parse_int(p);

    p = next_field(p);  /* Field 7: HDOP */
    if (p == NULL) return GPS_ERR_PARSE;
    double hdop = parse_double(p);

    p = next_field(p);  /* Field 8: Altitude */
    if (p == NULL) return GPS_ERR_PARSE;
    double alt = parse_double(p);

    data->fix_quality = (GpsFix_e)fix;
    data->satellites  = (uint8_t)sats;
    data->hdop        = hdop;

    if (fix > 0) {
        data->latitude       = nmea_to_degrees(raw_lat, ns);
        data->longitude      = nmea_to_degrees(raw_lon, ew);
        data->altitude       = alt;
        data->position_valid = true;
    }

    return GPS_OK;
}

/**
 * @brief  Validates checksum and dispatches to the correct parser.
 */
static GpsStatus_e parse_sentence(const char *sentence, GpsData_t *data)
{
    if (sentence[0] != '$') return GPS_ERR_PARSE;

    const char *star = strchr(sentence, '*');
    if (star == NULL) return GPS_ERR_PARSE;

    uint16_t body_len = (uint16_t)(star - sentence - 1U);
    uint8_t computed  = nmea_checksum(sentence + 1, body_len);
    uint8_t received  = (uint8_t)((hex_to_val(star[1]) << 4U) | hex_to_val(star[2]));

    if (computed != received) return GPS_ERR_CHECKSUM;

    const char *id = sentence + 3;
    const char *body = strchr(sentence, ',');
    if (body == NULL) return GPS_ERR_PARSE;
    body++;

    if (strncmp(id, "RMC", 3U) == 0) return parse_rmc(body, data);
    if (strncmp(id, "GGA", 3U) == 0) return parse_gga(body, data);

    return GPS_OK;
}

/* ========================  PUBLIC FUNCTIONS  =============================== */

void Gps_ForceOn(void)
{
    HAL_GPIO_WritePin(GPS_FORCE_ON_PORT, GPS_FORCE_ON_PIN, GPIO_PIN_SET);
}

void Gps_ForceOff(void)
{
    HAL_GPIO_WritePin(GPS_FORCE_ON_PORT, GPS_FORCE_ON_PIN, GPIO_PIN_RESET);
}

GpsStatus_e Gps_Init(Gps_Handle_t *h)
{
    if (h == NULL) {
        Log_Print("GPS", "Error: handle NULL en Init");
        return GPS_ERR_PARAM;
    }

    Log_Print("GPS", "Iniciando modulo GPS L86-M33...");

    Gps_ForceOn();
    HAL_Delay(100U);

    h->huart          = GPS_UART;
    h->sentence_idx   = 0U;
    h->sentence_ready = false;
    h->rx_byte        = 0U;

    memset(h->sentence, 0, GPS_SENTENCE_MAX_LEN);
    memset(&h->data, 0, sizeof(GpsData_t));

#ifdef GPS_DEBUG
    memset(&h->debug, 0, sizeof(GpsDebug_t));
#endif

    HAL_UART_Receive_IT(h->huart, &h->rx_byte, 1U);

    Log_Print("GPS", "GPS inicializado correctamente");
    return GPS_OK;
}

GpsStatus_e Gps_Process(Gps_Handle_t *h)
{
    if (h == NULL) return GPS_ERR_PARAM;
    if (!h->sentence_ready) return GPS_ERR_NO_FIX;

    char local[GPS_SENTENCE_MAX_LEN];
    memcpy(local, h->sentence, GPS_SENTENCE_MAX_LEN);
    h->sentence_ready = false;
    h->sentence_idx   = 0U;

    GpsStatus_e st = parse_sentence(local, &h->data);

#ifdef GPS_DEBUG
    if (st == GPS_OK) {
        h->debug.sentences_ok++;
        memcpy(h->debug.last_sentence, local, GPS_SENTENCE_MAX_LEN);
    } else if (st == GPS_ERR_CHECKSUM) {
        h->debug.sentences_fail++;
    }
#endif

    if (st == GPS_OK && h->data.position_valid) {
        Gps_FormatPosition(h);
    }

    return st;
}

void Gps_StoreByte(Gps_Handle_t *h)
{
    if (h == NULL) return;

    char c = (char)h->rx_byte;

#ifdef GPS_DEBUG
    h->debug.chars_processed++;
    h->debug.raw[h->debug.raw_idx & 0xFFU] = c;
    h->debug.raw_idx++;
#endif

    if (c == '$') {
        h->sentence_idx   = 0U;
        h->sentence_ready = false;
    }

    if (h->sentence_idx < (GPS_SENTENCE_MAX_LEN - 1U)) {
        h->sentence[h->sentence_idx] = c;
        h->sentence_idx++;
    }

    if (c == '\n') {
        h->sentence[h->sentence_idx] = '\0';
        h->sentence_ready = true;
    }

    HAL_UART_Receive_IT(h->huart, &h->rx_byte, 1U);
}

GpsStatus_e Gps_SendMTK(Gps_Handle_t *h, const char *cmd)
{
    if (h == NULL || cmd == NULL) return GPS_ERR_PARAM;

    uint16_t len = (uint16_t)strlen(cmd);
    uint8_t cs   = nmea_checksum(cmd, len);

    char buf[GPS_SENTENCE_MAX_LEN];
    int n = snprintf(buf, sizeof(buf), "$%s*%02X\r\n", cmd, cs);
    if (n <= 0 || (uint16_t)n >= sizeof(buf)) return GPS_ERR_OVERFLOW;

    HAL_StatusTypeDef hal = HAL_UART_Transmit(h->huart, (uint8_t *)buf,
                                               (uint16_t)n, GPS_TX_TIMEOUT_MS);
    if (hal != HAL_OK) {
        return (hal == HAL_TIMEOUT) ? GPS_ERR_TIMEOUT : GPS_ERR_UART;
    }

    return GPS_OK;
}

bool Gps_CheckWiring(Gps_Handle_t *h)
{
    if (h == NULL) return false;

#ifdef GPS_DEBUG
    if (h->debug.chars_processed < GPS_WIRING_MIN_CHARS) {
        return (h->debug.chars_processed > 0U);
    }
#endif

    return true;
}

void Gps_Reset(Gps_Handle_t *h)
{
    if (h == NULL) return;

    h->sentence_idx   = 0U;
    h->sentence_ready = false;
    memset(h->sentence, 0, GPS_SENTENCE_MAX_LEN);
    memset(&h->data, 0, sizeof(GpsData_t));
}

void Gps_FormatPosition(Gps_Handle_t *h)
{
    if (h == NULL) return;

    snprintf(h->data.position_str, GPS_POSITION_STR_LEN,
             "%.6f,%.6f", h->data.latitude, h->data.longitude);
}

/* ========================  DISTANCE TRACKING  ============================== */

#define GPS_TRACK_MIN_DIST_M    8.0
#define GPS_TRACK_MIN_SPEED_KMH 2.0
#define GPS_TRACK_MAX_DIST_M    80.0
#define GPS_EARTH_RADIUS_M      6371000.0
#define GPS_DEG2RAD(d)          ((d) * M_PI / 180.0)
#define GPS_COORD_ROUND_4DEC    10000.0    /* round to 4 decimal places (~11 m) */

static double haversine_m(double lat1, double lon1, double lat2, double lon2)
{
    double dlat = GPS_DEG2RAD(lat2 - lat1);
    double dlon = GPS_DEG2RAD(lon2 - lon1);
    double a = sin(dlat / 2.0) * sin(dlat / 2.0)
             + cos(GPS_DEG2RAD(lat1)) * cos(GPS_DEG2RAD(lat2))
             * sin(dlon / 2.0) * sin(dlon / 2.0);
    return GPS_EARTH_RADIUS_M * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

GpsStatus_e Gps_StartTracking(Gps_Handle_t *h)
{
    if (h == NULL) return GPS_ERR_PARAM;
    //if (!h->data.position_valid) return GPS_ERR_NO_FIX;

    h->tracking_active = true;
    h->total_distance  = 0.0f;
    h->last_lat = round(h->data.latitude  * GPS_COORD_ROUND_4DEC) / GPS_COORD_ROUND_4DEC;
    h->last_lon = round(h->data.longitude * GPS_COORD_ROUND_4DEC) / GPS_COORD_ROUND_4DEC;

    return GPS_OK;
}

void Gps_UpdateTracking(Gps_Handle_t *h)
{
    if (h == NULL) return;
    if (!h->tracking_active) return;
    if (!h->data.position_valid) return;

    /* Truncate to 5 decimal places (~1.1 m resolution) to suppress GPS jitter
     * that causes phantom movement while stationary. */
    double cur_lat = round(h->data.latitude  * GPS_COORD_ROUND_4DEC) / GPS_COORD_ROUND_4DEC;
    double cur_lon = round(h->data.longitude * GPS_COORD_ROUND_4DEC) / GPS_COORD_ROUND_4DEC;

    if (h->data.speed_kmh < GPS_TRACK_MIN_SPEED_KMH) return;

    double dist = haversine_m(h->last_lat, h->last_lon, cur_lat, cur_lon);

    if (dist >= GPS_TRACK_MIN_DIST_M && dist <= GPS_TRACK_MAX_DIST_M) {
        h->total_distance += (float)dist;
        h->last_lat = cur_lat;
        h->last_lon = cur_lon;
    }
}

void Gps_StopTracking(Gps_Handle_t *h)
{
    if (h == NULL) return;
    h->tracking_active = false;
}
