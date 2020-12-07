//-----------------------------------------------------------------------------
// Copyright (C) 2020 sirloins
//
// This code is licensed to you under the terms of the GNU GPL, version 2 or,
// at your option, any later version. See the LICENSE.txt file for the text of
// the license.
//-----------------------------------------------------------------------------
// Low frequency EM4x70 structs
//-----------------------------------------------------------------------------

#ifndef EM4X70_H__
#define EM4X70_H__

typedef struct {
    bool parity;
    bool address_given;
    bool pin_given;
    uint8_t address;
    uint8_t word[4];
    uint8_t pin[4];

} PACKED em4x70_data_t;

#endif /* EM4X70_H__ */
