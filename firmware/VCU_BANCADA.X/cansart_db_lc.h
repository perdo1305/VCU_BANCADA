/* 
 * File:   cansart_db_lc.h
 * Author: pedro
 *
 * Created on 6 de Março de 2024, 18:24
 */

#ifndef CANSART_DB_LC_H
#define	CANSART_DB_LC_H

#include <stdint.h>

#define SLAVEMODE 1

struct frame10 {
    uint8_t ID;
    uint8_t ADC0H;
    uint8_t ADC0L;
    uint8_t ADC3H;
    uint8_t ADC3L;
    uint8_t DATA5;
    uint8_t DATA6;
    uint8_t DATA7;
    uint8_t DATA8;
    uint8_t LENGHT;
};
/*
struct frame23 {
    uint8_t ID;
    uint8_t TEMP;
    uint8_t OIL;
    uint8_t DATA3;
    uint8_t DATA4;
    uint8_t DATA5;
    uint8_t DATA6;
    uint8_t DATA7;
    uint8_t DATA8;
    uint8_t LENGHT;
};

struct frame121 {
    uint8_t ID;
    uint8_t SetRPM;
    uint8_t SetPower;
    uint8_t DATA3;
    uint8_t DATA4;
    uint8_t DATA5;
    uint8_t DATA6;
    uint8_t DATA7;
    uint8_t DATA8;
    uint8_t LENGHT;
};*/

#endif