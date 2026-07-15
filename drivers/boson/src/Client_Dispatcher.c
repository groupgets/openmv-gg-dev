//  /////////////////////////////////////////////////////
//  // DO NOT EDIT.  This is a machine generated file. //
//  /////////////////////////////////////////////////////
/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/*  This document is controlled to FLIR Technology Level 2. The information   */
/*  contained in this document pertains to a dual use product controlled for  */
/*  export by the Export Administration Regulations (EAR). Diversion contrary */
/*  to US law is prohibited. US Department of Commerce authorization is not   */
/*  required prior to export or transfer to foreign persons or parties unless */
/*  otherwise prohibited.                                                     */
/*                                                                            */
/******************************************************************************/

#include "Client_Dispatcher.h"

// Asynchronous (MultiService compatible) transmit part
FLR_RESULT CLIENT_dispatcher_Tx(uint32_t seqNum, FLR_FUNCTION fnID, const uint8_t *sendData, const uint32_t sendBytes, const uint8_t *receiveData, uint32_t *receiveBytes) {
    
    uint32_t i;
    
    // Allocated buffer with extra space for payload header
    uint8_t sendPayload[530];
    uint8_t *pyldPtr = (uint8_t *)sendPayload;
    
    // Write sequence number to first 4 bytes
    UINT_32ToByte(seqNum, (const uint8_t *)pyldPtr);
    pyldPtr += 4;
    
    // Write function ID to second 4 bytes
    UINT_32ToByte((const uint32_t) fnID, (const uint8_t *)pyldPtr);
    pyldPtr += 4;
    
    // Write 0xFFFFFFFF to third 4 bytes
    UINT_32ToByte(0xFFFFFFFF, (const uint8_t *)pyldPtr);
    pyldPtr += 4;
    
    // Copy sendData to payload buffer
    uint8_t *dataPtr = (uint8_t *)sendData;
    for(i = 0;i<sendBytes;i++) {
        *pyldPtr++ = *dataPtr++;
    }
    
    if(CLIENT_interface_writeFrame(sendPayload, sendBytes + 12) != FLR_OK)
        return FLR_COMM_ERROR_WRITING_COMM;
    
    return R_SUCCESS;
}
// Asynchronous (MultiService compatible) receive part
FLR_RESULT CLIENT_dispatcher_Rx(uint32_t *seqNum, uint32_t *fnID, const uint8_t *sendData, const uint32_t sendBytes, const uint8_t *receiveData, uint32_t *receiveBytes) {

    uint32_t i;

    // Allocated buffer with extra space for return data
    uint8_t receivePayload[530];
    uint8_t *inPtr = (uint8_t *)receivePayload;

    // OPENMV: the caller's buffer holds this many payload bytes. A frame
    // other than the expected response (a stale late reply from a command
    // that timed out) can be larger; the copy below must never exceed it.
    const uint32_t callerCapacity = *receiveBytes;

    *receiveBytes+=12;
    if(CLIENT_interface_readFrame(receivePayload, receiveBytes) != FLR_OK)
        return FLR_COMM_ERROR_READING_COMM;
    
    if (*receiveBytes < 12) {
        if(CLIENT_interface_readFrame(receivePayload, receiveBytes) != FLR_OK)
            return FLR_COMM_ERROR_READING_COMM;
    }
    
    if (*receiveBytes < 12)
        return FLR_COMM_ERROR_READING_COMM;
    
    // Evaluate sequence bytes as UINT_32
    uint32_t returnSequence;
    byteToUINT_32( (const uint8_t *) inPtr, &returnSequence);
    inPtr += 4;
    
    // Ensure that received sequence matches sent sequence
    if(seqNum){
        *seqNum = returnSequence;
    }
    
    // Evaluate CMD ID bytes as UINT_32 
    uint32_t cmdID;
    byteToUINT_32( (const uint8_t *) inPtr, &cmdID);
    inPtr += 4;
    
    // Ensure that received CMD ID matches sent CMD ID
    if(fnID){
        *fnID = cmdID;
    }
    
    // Evaluate Payload Status bytes as UINT_32
    uint32_t pyldStatus;
    byteToUINT_32( (const uint8_t *) inPtr, &pyldStatus);
    inPtr += 4;
    
    const FLR_RESULT returncode = (FLR_RESULT) pyldStatus;
    // Check for any errorcode
    if(returncode != R_SUCCESS){
        return returncode;
    }
    
    // Now have Good Tx, Good Sequence, Good CMD ID, and Good Status.
    // inPtr at Data block, fill receiveData buffer with outPtr
    uint8_t *outPtr = (uint8_t *)receiveData;
    // decrement receiveBytes by 12 (len of header bytes)
    *receiveBytes-=12;

    uint32_t localvar = *receiveBytes; //shouldn't have to do this, but it works.
    // OPENMV: never copy past the caller's buffer — an unexpected (stale)
    // frame larger than the expected response otherwise corrupts the stack.
    if (localvar > callerCapacity) {
        localvar = callerCapacity;
        *receiveBytes = callerCapacity;
    }
    for(i=0;i<localvar;i++) {
        *outPtr++ = *inPtr++;
    }

    return R_SUCCESS;
} // End CLIENT_dispatcher()

// Synchronous (potentially MultiService incompatible) transmit+receive variant
FLR_RESULT CLIENT_dispatcher(uint32_t seqNum, FLR_FUNCTION fnID, const uint8_t *sendData, const uint32_t sendBytes, const uint8_t *receiveData, uint32_t *receiveBytes)
{    uint32_t returnSequence;
    uint32_t cmdID;
    FLR_RESULT res = CLIENT_dispatcher_Tx(seqNum, fnID, sendData, sendBytes, receiveData, receiveBytes);
    if (res)
        return res;

    // OPENMV: a response that arrives after the read timeout stays queued in
    // the receive path (the SC16IS741A bridge FIFO holds it indefinitely), so
    // the NEXT command would read its predecessor's response and the channel
    // wedges off-by-one until reboot — every later command fails with a
    // sequence mismatch, or worse, inherits a stale frame's NAK status
    // (checked below only AFTER the sequence matches, for the same reason).
    // Seen in the field on a hot Boson 320 whose auto-FFC/gain-switch stalls
    // push replies past the timeout. Drain stale frames: re-read until the
    // sequence matches the command just sent, bounded so a flood of garbage
    // cannot spin here forever.
    const uint32_t expectBytes = *receiveBytes;
    uint32_t stale = 0;
    for (;;) {
        *receiveBytes = expectBytes;
        returnSequence = ~seqNum;
        cmdID = (uint32_t) fnID;
        res = CLIENT_dispatcher_Rx(&returnSequence, &cmdID, sendData, sendBytes, receiveData, receiveBytes);
        if (res == FLR_COMM_ERROR_READING_COMM)
            return res;             // nothing (more) buffered — a real timeout
        if (returnSequence == seqNum)
            break;                  // our response, whatever its status
        if (++stale > 3)
            return R_SDK_DSPCH_SEQUENCE_MISMATCH;
    }
    if (res)
        return res;

    if (cmdID ^ (uint32_t) fnID)
        return R_SDK_DSPCH_ID_MISMATCH;

    return R_SUCCESS;
}

FLR_RESULT CheckReadyDataCommandId(uint32_t receiveBytes, const uint8_t *receiveData, uint32_t *commandId)
{
    if (!receiveData || !commandId)
        return FLR_ERROR;
    
    if (receiveBytes < 8)
        return FLR_RANGE_ERROR;
    
    uint8_t *inPtr = (uint8_t *)receiveData;
    
    // Evaluate sequence bytes as UINT_32
    uint32_t returnSequence;
    byteToUINT_32( (const uint8_t *) inPtr, &returnSequence);
    inPtr += 4;
    
    // Evaluate CMD ID bytes as UINT_32
    uint32_t cmdID;
    byteToUINT_32( (const uint8_t *) inPtr, &cmdID);
    
    *commandId = cmdID;
    
    return R_SUCCESS;
}
