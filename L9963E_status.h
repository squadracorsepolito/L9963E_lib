/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy us a beer in return.
 * 
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 */

#ifndef L9963E_STATUS_H
#define L9963E_STATUS_H

typedef enum {
    L9963E_OK = 0,
    L9963E_ERROR,
    L9963E_TIMEOUT,
    L9963E_CRC_ERROR,
    L9963E_READBACK_ERROR
} L9963E_StatusTypeDef;

#endif  // L9963E_STATUS_H