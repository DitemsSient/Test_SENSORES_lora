/**
 * @file    HWTest_Status.h
 * @brief   Global hardware test result structure.
 *
 * @details Holds one boolean per testable component. Set to true on pass,
 *          false on fail — both for automatic tests (_Test() functions) and
 *          manual confirmations (user presses SI / NO in the menu).
 *
 *          Add a new field here whenever a new component is added to the
 *          test suite. For a new PCB, add a new sub-struct alongside HWMira_t.
 *
 *          Usage:
 *            main.c   → declare:  HWTest_Status_t hw_status;
 *            others   → access:   extern HWTest_Status_t hw_status;
 *
 * @date    May 2026
 * @author  César Pérez
 * @version 1.0.0
 */

#ifndef HWTEST_STATUS_H
#define HWTEST_STATUS_H

#include <stdbool.h>

/* ============================  STRUCTURES  ================================ */

/* Test results for PCB Mira */

typedef struct {
    /* --- Automatic tests (Sensores) --- */
    bool flash;
    bool hall;
    bool luz_ambiental;
    bool rs485;
    bool bluetooth;
    bool imu;           /**< LSM6DSO32TR + MMC5983MA: ambos deben pasar       */
    /* --- Manual tests --- */
    bool buzzer;
    bool vibrador;
    bool rgb_driver;
} HWMira_t;

/* Test results for PCB Sensores (remote via BT) */

typedef struct {
    /* --- Automatic tests (received from BT: $GPS,LORA,FLASH,IMU\r) --- */
    bool gps;
    bool lora;
    bool flash;
    bool imu;
    /* --- Manual tests --- */
    bool buzzer;
    bool rgb;
} HWSensores_t;

/* Top-level container — one sub-struct per PCB */

typedef struct {
    HWMira_t     mira;
    HWSensores_t sensores;
} HWTest_Status_t;

#endif /* HWTEST_STATUS_H */
