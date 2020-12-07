//-----------------------------------------------------------------------------
// Copyright (C) 2020 sirloins
//
// This code is licensed to you under the terms of the GNU GPL, version 2 or,
// at your option, any later version. See the LICENSE.txt file for the text of
// the license.
//-----------------------------------------------------------------------------
// Low frequency EM4x70 commands
//-----------------------------------------------------------------------------

#include "cmdlfem4x70.h"
#include <ctype.h>
#include "cmdparser.h"    // command_t
#include "fileutils.h"
#include "comms.h"
#include "commonutil.h"
#include "em4x70.h"


static int CmdHelp(const char *Cmd);


static int usage_lf_em4x70_info(void) {
    PrintAndLogEx(NORMAL, "Read all information of EM4x70. Tag must be on antenna.");
    PrintAndLogEx(NORMAL, "");
    PrintAndLogEx(NORMAL, "Usage:  lf em 4x70 info [h]");
    PrintAndLogEx(NORMAL, "Options:");
    PrintAndLogEx(NORMAL, "       h         - this help");
    PrintAndLogEx(NORMAL, "       p         - use even parity for commands");
    PrintAndLogEx(NORMAL, "Examples:");
    PrintAndLogEx(NORMAL, _YELLOW_("      lf em 4x70_info"));
    PrintAndLogEx(NORMAL, "");
    return PM3_SUCCESS;
}

static int usage_lf_em4x70_write(void) {
    PrintAndLogEx(NORMAL, "Write EM4x70 word. Tag must be on antenna. ");
    PrintAndLogEx(NORMAL, "");
    PrintAndLogEx(NORMAL, "Usage:  lf em 4x70 write [h] [a <address>] [w <data>]");
    PrintAndLogEx(NORMAL, "Options:");
    PrintAndLogEx(NORMAL, "       h         - this help");
    PrintAndLogEx(NORMAL, "       a <addr>  - memory address to write to (dec)");
    PrintAndLogEx(NORMAL, "       w <word>  - word to write (hex)");
    PrintAndLogEx(NORMAL, "Examples:");
    PrintAndLogEx(NORMAL, _YELLOW_("      lf em 4x70_write a 3 w c0de"));
    PrintAndLogEx(NORMAL, "");
    return PM3_SUCCESS;
}

static int usage_lf_em4x70_send_pin(void) {
    PrintAndLogEx(NORMAL, "Send pin to tag, unlocking it. Tag must be on antenna. ");
    PrintAndLogEx(NORMAL, "");
    PrintAndLogEx(NORMAL, "Usage:  lf em 4x70 send_pin [h] [p <pin>]");
    PrintAndLogEx(NORMAL, "Options:");
    PrintAndLogEx(NORMAL, "       h         - this help");
    PrintAndLogEx(NORMAL, "       p <pinr>  - pin code (4 bytes hex)");
    PrintAndLogEx(NORMAL, "Examples:");
    PrintAndLogEx(NORMAL, _YELLOW_("      lf em 4x70_send_pin p 11223344"));
    PrintAndLogEx(NORMAL, "");
    return PM3_SUCCESS;
}

static void print_info_result(uint8_t *data) {

    PrintAndLogEx(NORMAL, "");
    PrintAndLogEx(INFO, "--- " _CYAN_("Tag Information") " ---------------------------");
    PrintAndLogEx(INFO, "-------------------------------------------------------------");

    // data section
    PrintAndLogEx(NORMAL, "");
    PrintAndLogEx(INFO, _YELLOW_("EM4x70 data:"));
    
    for(int i=1; i <= 32; i+=2) {
        PrintAndLogEx(NORMAL, "%02X %02X", data[32-i], data[32-i-1]);
    }
    PrintAndLogEx(NORMAL, "Tag ID: %02X %02X %02X %02X", data[7], data[6], data[5], data[4]);
    PrintAndLogEx(NORMAL, "Lockbit 0: %d", (data[3] & 0x40) ? 1:0);
    PrintAndLogEx(NORMAL, "Lockbit 1: %d", (data[3] & 0x80) ? 1:0);
    PrintAndLogEx(NORMAL, "");

}

int em4x70_info(void) {
    
    em4x70_data_t edata = {
        .parity = false // TODO: try both? or default to true
    };

    clearCommandBuffer();
    SendCommandNG(CMD_LF_EM4X70_INFO, (uint8_t *)&edata, sizeof(edata));
    
    PacketResponseNG resp;
    if (!WaitForResponseTimeout(CMD_LF_EM4X70_INFO, &resp, TIMEOUT)) {
        PrintAndLogEx(WARNING, "(em4x70) timeout while waiting for reply.");
        return PM3_ETIMEOUT;
    }

    if (resp.status) {
        print_info_result(resp.data.asBytes);
        return PM3_SUCCESS;
    }

    return PM3_ESOFT;
}

//quick test for EM4x70 tag
bool detect_4x70_block(void) {

    return em4x70_info() == PM3_SUCCESS;
}

int CmdEM4x70Info(const char *Cmd) {

    // envoke reading of a EM4x70 tag which has to be on the antenna because
    // decoding is done by the device (not on client side)

    bool errors = false;
    uint8_t cmdp = 0;

    em4x70_data_t etd = {0};

    while (param_getchar(Cmd, cmdp) != 0x00 && !errors) {
        switch (tolower(param_getchar(Cmd, cmdp))) {

            case 'h':
                return usage_lf_em4x70_info();
            
            case 'p':
                etd.parity = true;
                cmdp +=1;
                break;

            default:
                PrintAndLogEx(WARNING, "  Unknown parameter '%c'", param_getchar(Cmd, cmdp));
                errors = true;
                break;
        }
    }

    // validation
    if (errors)
        return usage_lf_em4x70_info();

    clearCommandBuffer();
    SendCommandNG(CMD_LF_EM4X70_INFO, (uint8_t *)&etd, sizeof(etd));

    PacketResponseNG resp;
    if (!WaitForResponseTimeout(CMD_LF_EM4X70_INFO, &resp, TIMEOUT)) {
        PrintAndLogEx(WARNING, "timeout while waiting for reply.");
        return PM3_ETIMEOUT;
    }

    if (resp.status) {
        print_info_result(resp.data.asBytes);
        return PM3_SUCCESS;
    }

    PrintAndLogEx(FAILED, "reading tag " _RED_("failed"));
    return PM3_ESOFT;
}

int CmdEM4x70Write(const char *Cmd) {

    // write one word (16 bits) to the tag at given address (0-15)

    bool errors = false, bword = false, baddr = false;
    uint8_t cmdp = 0;

    em4x70_data_t etd = {0};

    while (param_getchar(Cmd, cmdp) != 0x00 && !errors) {
        switch (tolower(param_getchar(Cmd, cmdp))) {

            case 'h':
                return usage_lf_em4x70_write();
            
            case 'p':
                etd.parity = true;
                cmdp +=1;
                break;
            
            case 'w': {
                if (param_gethex(Cmd, cmdp + 1, etd.word, 4)) {
                    PrintAndLogEx(FAILED, "\n  word has to be 4 hex symbols\n");
                    return PM3_EINVARG;
                }
                bword = true;
                cmdp += 2;
                break;
            }
            
            case 'a': {
                param_getdec(Cmd, cmdp + 1, &etd.address);

                // validation
                if (etd.address > 15) {
                    PrintAndLogEx(FAILED, "\n  error, address has to be in range [0-15]\n");
                    return PM3_EINVARG;
                }
                baddr = true;
                cmdp += 2;
                break;
            }
            
            default:
                PrintAndLogEx(WARNING, "  Unknown parameter '%c'", param_getchar(Cmd, cmdp));
                errors = true;
                break;
        }
    }

    // validation
    if (errors || !bword || !baddr)
        return usage_lf_em4x70_write();

    clearCommandBuffer();
    SendCommandNG(CMD_LF_EM4X70_WRITE, (uint8_t *)&etd, sizeof(etd));

    PacketResponseNG resp;
    if (!WaitForResponseTimeout(CMD_LF_EM4X70_WRITE, &resp, TIMEOUT)) {
        PrintAndLogEx(WARNING, "timeout while waiting for reply.");
        return PM3_ETIMEOUT;
    }

    if (resp.status) {
        print_info_result(resp.data.asBytes);
        return PM3_SUCCESS;
    }

    PrintAndLogEx(FAILED, "reading tag " _RED_("failed"));
    return PM3_ESOFT;
}

int CmdEM4x70SendPin(const char *Cmd) {

    // send pin code to device, unlocking it for writing

    bool errors = false, bpin = false;
    uint8_t cmdp = 0;

    em4x70_data_t etd = {0};

    while (param_getchar(Cmd, cmdp) != 0x00 && !errors) {
        switch (tolower(param_getchar(Cmd, cmdp))) {

            case 'h':
                return usage_lf_em4x70_send_pin();
            
            case 'p': {
                if (param_gethex(Cmd, cmdp + 1, etd.pin, 8)) {
                    PrintAndLogEx(FAILED, "\n  pin has to be 8 hex symbols\n");
                    return PM3_EINVARG;
                }
                etd.pin_given = true;
                bpin = true;
                cmdp += 2;
                break;
            }
            
            default:
                PrintAndLogEx(WARNING, "  Unknown parameter '%c'", param_getchar(Cmd, cmdp));
                errors = true;
                break;
        }
    }

    // validation
    if (errors || !bpin)
        return usage_lf_em4x70_send_pin();

    clearCommandBuffer();
    SendCommandNG(CMD_LF_EM4X70_SEND_PIN, (uint8_t *)&etd, sizeof(etd));

    PacketResponseNG resp;
    if (!WaitForResponseTimeout(CMD_LF_EM4X70_SEND_PIN, &resp, TIMEOUT)) {
        PrintAndLogEx(WARNING, "timeout while waiting for reply.");
        return PM3_ETIMEOUT;
    }

    if (resp.status) {
        print_info_result(resp.data.asBytes);
        return PM3_SUCCESS;
    }

    PrintAndLogEx(FAILED, "reading tag " _RED_("failed"));
    return PM3_ESOFT;
}

static command_t CommandTable[] = {
    {"help",     CmdHelp,           AlwaysAvailable, "This help"},
    {"info",     CmdEM4x70Info,     IfPm3EM4x70,     "tag information EM4x70"},
    {"write",    CmdEM4x70Write,    IfPm3EM4x70,     "write EM4x70"},
    {"send_pin", CmdEM4x70SendPin,  IfPm3EM4x70,     "send pin EM4x70"},
    {NULL, NULL, NULL, NULL}
};

static int CmdHelp(const char *Cmd) {
    (void)Cmd; // Cmd is not used so far
    CmdsHelp(CommandTable);
    return PM3_SUCCESS;
}

int CmdLFEM4X70(const char *Cmd) {
    clearCommandBuffer();
    return CmdsParse(CommandTable, Cmd);
}
