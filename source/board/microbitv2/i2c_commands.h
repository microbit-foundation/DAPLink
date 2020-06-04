/**
 * @file i2c_commands.h
 * @brief
 */

#ifndef I2C_COMMANDS_H
#define I2C_COMMANDS_H

#include "cmsis_compiler.h"

#ifdef __cplusplus
extern "C" {
#endif

/*! i2c command Id type enumeration */
typedef enum cmdId_tag {
    gReadRequest_c  = 0x10,
    gReadResponse_c = 0x11,
    gWriteRequest_c = 0x12,
    gWriteResponse_c = 0x13,
    gErrorResponse_c = 0x20
} cmdId_t;

/*! property Id type enumeration */
typedef enum propertyId_tag {
    gDAPLinkBoardVersion_c  = 0x01,
    gI2CProtocolVersion_c   = 0x02,
    gDAPLinkVersion_c       = 0x03,
    gPowerState_c           = 0x04,
    gPowerConsumption_c     = 0x05,
    gUSBEnumerationState_c  = 0x06,
    gPowerMode_c            = 0x07,
    gNRFPowerMode_c         = 0x08
} propertyId_t;

/*! property Id type enumeration */
typedef enum errorCode_tag {
    gErrorSuccess               = 0x30,
    gErrorIncompleteCommand_c   = 0x31,
    gErrorUnknownCommand_c      = 0x32,
    gErrorCommandDisallowed_c   = 0x33,
    gErrorUnknownProperty_c     = 0x34,
    gErrorWrongPropertySize_c   = 0x35,
    gErrorReadDisallowed_c      = 0x36,
    gErrorWriteDisallowed_c     = 0x37,
    gErrorWriteFail_c           = 0x38
} errorCode_t;

typedef __PACKED_STRUCT readReqCmd_tag {
    uint8_t     propertyId;
} readReqCmd_t;

#define MAX_PROPERTY_SIZE   8

typedef __PACKED_STRUCT readRspCmd_tag {
    uint8_t     propertyId;
    uint8_t     dataSize;
    uint8_t     data[MAX_PROPERTY_SIZE];
} readRspCmd_t;

typedef __PACKED_STRUCT writeReqCmd_tag {
    uint8_t     propertyId;
    uint8_t     dataSize;
    uint8_t     data[MAX_PROPERTY_SIZE];
} writeReqCmd_t;

typedef __PACKED_STRUCT writeRspCmd_tag {
    uint8_t     propertyId;
} writeRspCmd_t;

typedef __PACKED_STRUCT errorRspCmd_tag {
    uint8_t     errorCode;
} errorRspCmd_t;

/*! i2c command structure*/
typedef __PACKED_STRUCT i2cCommand_tag {
    uint8_t cmdId;
    __PACKED_UNION {
        readReqCmd_t    readReqCmd;
        readRspCmd_t    readRspCmd;
        writeReqCmd_t   writeReqCmd;
        writeRspCmd_t   writeRspCmd;
        errorRspCmd_t   errorRspCmd;
    } cmdData;
} i2cCommand_t;


#ifdef __cplusplus
}
#endif

#endif
