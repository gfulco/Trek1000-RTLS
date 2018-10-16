/*! ----------------------------------------------------------------------------
 *  @file    instance_anch.c
 *  @brief   Decawave anchor application state machine and for TREK demo
 *
 * @attention
 *
 * Copyright 2016 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include "compiler.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_spi.h"
#include "deca_regs.h"

#include "instance.h"

// -------------------------------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------------------------------
//      Data Definitions
// -------------------------------------------------------------------------------------------------------------------

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// NOTE: the maximum RX timeout is ~ 65ms
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// -------------------------------------------------------------------------------------------------------------------
// Functions
// -------------------------------------------------------------------------------------------------------------------
void anch_prepare_anc2tag_response(unsigned int tof_index, uint8 srcAddr_index, uint8 fcode_index, uint8 *frame, uint32 uTimeStamp);
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
void anch_prepare_anc2anc_response(uint16 sourceAddress, uint8 srcAddr_index, uint8 fcode_index, uint8 *frame);
#endif
void anch_enable_rx(uint32 dlyTime);

/**
 * @brief this function re-enables RX ...
 */
void anch_no_timeout_rx_reenable(void)
{
	dwt_setrxtimeout(0); //reconfigure the timeout
	dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
}

#if (ANCTOANCTWR == 1) //anchor to anchor ranging
//go back to RX and set ANCHOR mode
void rnganch_change_back_to_anchor(instance_data_t *inst)
{
	//stay in RX and behave as anchor
	inst->testAppState = TA_RXE_WAIT ;
	inst->mode = ANCHOR ;
	inst->twrMode = LISTENER;
	led_off(LED_PC7);
	dwt_setrxtimeout(0);
	dwt_setrxaftertxdelay(0);
}

//change to ANCHOR_RNG mode
void anch_change_to_rnganchor(instance_data_t *inst)
{
	port_DisableEXT_IRQ(); //enable ScenSor IRQ before starting
	//anchor0 sends poll to anchor1
	inst->mode = ANCHOR_RNG; //change to ranging initiator
	inst->twrMode = INITIATOR;
	dwt_forcetrxoff(); //disable DW1000
	instance_clearevents(); //clear any events
	//change state to send a Poll
	inst->testAppState = TA_TXPOLL_WAIT_SEND ;
	led_on(LED_PC7);
	if(inst->gatewayAnchor)
	{
		inst->rangeNumAnc++; //A1 keeps same ranging seq as A0
		inst->remainingRespToRx = NUM_EXPECTED_RESPONSES_ANC0; //2 responses A1, A2 as A3 is not involved in Anc to Anc ranging

	}
	else
	{
		inst->remainingRespToRx = NUM_EXPECTED_RESPONSES_ANC1; //1 responses A2 as A3 is not involved in Anc to Anc ranging
	}

	port_EnableEXT_IRQ(); //enable ScenSor IRQ before starting
}

/**
 * @brief function to re-enable the receiver and also adjust the timeout before sending the final message
 * if it is time so send the final message, the event from the callback will notify the application,
 * else the receiver is automatically re-enabled
 *
 * this function is only used for anchors (having a role of ANCHOR_RNG) when ranging to other anchors
 */
uint8 rnganchrxresp_signalsendfinal_or_rx_reenable(void)
{
	uint8 typePend = DWT_SIG_DW_IDLE;
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	//application will decide if to send the final or go back to default mode
	if(inst->remainingRespToRx == 0)
	{
		typePend = DWT_SIG_DW_IDLE; //RX finished
	}
	else
	{
		uint32 delayTime = inst->txu.tagPollTxTime >> 8 ;
		//still awaiting to receive more responses
		//if A0 got Response from A1 - go back to wait for next anchor's response (i.e. A2)
		dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy); //reconfigure the timeout

		if(inst->gatewayAnchor)
		{
			anch_enable_rx(delayTime + 2*inst->fixedReplyDelayAnc32h);
		}
		else
		{
			//should not get here, as A1 will go into first if (on reception of A2's response)
			//and A0 is gateway (do delayed RX above)
			dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
		}
		typePend = DWT_SIG_RX_PENDING ;
	}

	return typePend;
}
/**
 * @brief this function either re-enables the receiver (delayed or immediate) or transmits the response frame
 *
 * @param the sourceAddress is the address of the sender of the current received frame
 * @param time of reception of the poll from initiating ranging anchor
 *
 */
uint8 rnganch_txresponse_or_rx_reenable(uint16 sourceAddress, uint32 pollrxtime)
{
	uint8 typePend = DWT_SIG_DW_IDLE;
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	//set the reply time (the actual will be Poll rx time + inst->fixedReplyDelayAnc)
	inst->delayedTRXTime32h =  pollrxtime + inst->fixedReplyDelayAnc32h;

	//need longer delay for A2's response
	if((inst->instanceAddress16 == A2_ANCHOR_ADDR) && (sourceAddress == GATEWAY_ANCHOR_ADDR))
		 //this is A2 and Gateway Poll has just been received,
	{    //so after responding need to enable delayed RX to receive Final
		inst->delayedTRXTime32h += inst->fixedReplyDelayAnc32h;
		dwt_setrxaftertxdelay(inst->anc2RespTx2FinalRxDelay_sy);
		dwt_setrxtimeout(inst->fwto4FinalFrame_sy);
		inst->wait4final = WAIT4ANCFINAL;
	}
	else //this is A1 and Gateway Poll has just been received, or this is A2 and A1's Poll has just been received
	{    //so after responding need to enable delayed RX to receive Final
		inst->wait4final = WAIT4ANCFINAL;
		dwt_setrxtimeout(inst->fwto4FinalFrame_sy);
		dwt_setrxaftertxdelay(inst->anc1RespTx2FinalRxDelay_sy);
	}

	//response (final) is expected
	inst->wait4ack = DWT_RESPONSE_EXPECTED; //re has/will be re-enabled

	dwt_setdelayedtrxtime(inst->delayedTRXTime32h) ;
	if(dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED))
	{
		//if TX has failed - we need to re-enable RX for the next response or final reception...
		dwt_setrxaftertxdelay(0);
		inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
		//inst->lateTX++;
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		typePend = DWT_SIG_RX_PENDING ;
	}
	else
	{
		typePend = DWT_SIG_TX_PENDING ; // exit this interrupt and notify the application/instance that TX is in progress.
		inst->timeofTx = portGetTickCnt();
		inst->monitor = 1;
	}

	return typePend;
}
/**
 * @brief this function prepares and writes the anchor to anchor response frame into the TX buffer
 * it is called after anchor receives a Poll from an anchor
 */
void anch_prepare_anc2anc_response(uint16 sourceAddress, uint8 srcAddr_index, uint8 fcode_index, uint8 *frame)
{
	uint16 frameLength = 0;
	uint8 tof_idx = (sourceAddress) & 0x3 ;
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	inst->psduLength = frameLength = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
	//set the destination address (copy source as this is a reply)
	memcpy(&inst->msg_f.destAddr[0], &frame[srcAddr_index], ADDR_BYTE_SIZE_S); //remember who to send the reply to (set destination address)
	inst->msg_f.sourceAddr[0] = inst->eui64[0];
	inst->msg_f.sourceAddr[1] = inst->eui64[1];
	// Write calculated TOF into response message (get the previous ToF+range number from that anchor)
	memcpy(&(inst->msg_f.messageData[TOFR]), &inst->tofAnc[tof_idx], 4);
	inst->msg_f.messageData[TOFRN] = inst->rangeNumAAnc[tof_idx]; //get the previous range number

	inst->rangeNumAAnc[tof_idx] = 0; //clear the entry
	inst->rangeNumAnc = frame[POLL_RNUM + fcode_index] ;
	inst->msg_f.seqNum = inst->frameSN++;

	//set the delayed rx on time (the final message will be sent after this delay)
	dwt_setrxaftertxdelay(inst->ancRespRxDelay_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

	inst->tagSleepCorrection_ms = 0;
	inst->msg_f.messageData[RES_TAG_SLP0] = 0 ;
	inst->msg_f.messageData[RES_TAG_SLP1] = 0 ;

	inst->msg_f.messageData[FCODE] = RTLS_DEMO_MSG_ANCH_RESP2; //message function code (specifies if message is a poll, response or other...)

	//write the TX data
	dwt_writetxfctrl(frameLength, 0, 1);
	dwt_writetxdata(frameLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data

}
#endif

#if (DISCOVERY == 1)
/**
 * @brief this function adds newly discovered tag to our system and assigns a slot to it
 */
int anch_add_tag_to_list(uint8 *tagAddr)
{
    instance_data_t* inst = instance_get_local_structure_ptr(0);
    uint8 i;
    uint8 blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int slot = -1;

    //add the new Tag to the list, if not already there and there is space
    for(i=0; i<MAX_TAG_LIST_SIZE; i++)
    {
        if(memcmp(&inst->tagList[i][0], &tagAddr[0], 8) != 0)
        {
            if(memcmp(&inst->tagList[i][0], &blank[0], 8) == 0) //blank entry
            {
                memcpy(&inst->tagList[i][0], &tagAddr[0], 8) ;
                inst->tagListLen = i + 1 ;
                slot = i;
                break;
            }
        }
        else
        {
			slot = i;
            break; //we already have this Tag in the list
        }
    }

    return slot;
}

/**
 * @brief this function prepares and writes the anchor to tag ranging init frame into the TX buffer
 * it is called after anchor receives a Blink from a tag
 */
int anch_prepare_anc2tag_rangeinitresponse(uint8 *tagID, uint8 slot, uint32 uTimeStamp)
{
	int typePend = DWT_SIG_DW_IDLE;
	uint16 frameLength = 0;
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	inst->psduLength = frameLength = RANGINGINIT_MSG_LEN + FRAME_CRTL_AND_ADDRESS_LS + FRAME_CRC;

	memcpy(&inst->rng_initmsg.destAddr[0], tagID, ADDR_BYTE_SIZE_L); //remember who to send the reply to (set destination address)
	inst->rng_initmsg.sourceAddr[0] = inst->eui64[0];
	inst->rng_initmsg.sourceAddr[1] = inst->eui64[1];

	inst->rng_initmsg.seqNum = inst->frameSN++;

	{
		int error = 0;
		int currentSlotTime = 0;
		int expectedSlotTime = 0;
		//find the time in the current superframe
		currentSlotTime = uTimeStamp % inst->sframePeriod_ms;

		//this is the slot time the poll should be received in (Mask 0x07 for the 8 MAX tags we support in TREK)
		expectedSlotTime = (slot) * inst->slotDuration_ms; //

		//error = expectedSlotTime - currentSlotTime
		error = expectedSlotTime - currentSlotTime;

		if(error < (-(inst->sframePeriod_ms>>1))) //if error is more negative than 0.5 period, add whole period to give up to 1.5 period sleep
		{
			inst->tagSleepCorrection_ms = (inst->sframePeriod_ms + error);
		}
		else //the minimum Sleep time will be 0.5 period
		{
			inst->tagSleepCorrection_ms = error;
		}

		inst->tagSleepCorrection_ms += inst->sframePeriod_ms ; //min is at least 1.5 periods

		inst->rng_initmsg.messageData[RES_TAG_SLP0] = inst->tagSleepCorrection_ms & 0xFF ;
		inst->rng_initmsg.messageData[RES_TAG_SLP1] = (inst->tagSleepCorrection_ms >> 8) & 0xFF;
	}

	inst->rng_initmsg.messageData[RES_TAG_ADD0] = slot & 0xFF;
	inst->rng_initmsg.messageData[RES_TAG_ADD1] = (slot >> 8) & 0xFF;

	inst->rng_initmsg.messageData[FCODE] = RTLS_DEMO_MSG_RNG_INIT; //message function code (specifies if message is a poll, response or other...)

	//write the TX data
	dwt_writetxfctrl(frameLength, 0, 1);
	dwt_writetxdata(frameLength, (uint8 *)  &inst->rng_initmsg, 0) ;	// write the frame data

	dwt_setrxaftertxdelay(0);

	//response is expected
	inst->wait4ack = DWT_RESPONSE_EXPECTED; //re has/will be re-enabled

	dwt_setdelayedtrxtime(inst->delayedTRXTime32h) ;

	if(dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED))
	{
		//if TX has failed - we need to re-enable RX for the next response or final reception...
		dwt_setrxaftertxdelay(0);
		inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
		//inst->lateTX++;
	}
	else
	{
		typePend = DWT_SIG_TX_PENDING ;
		inst->timeofTx = portGetTickCnt();
		inst->monitor = 1;
	}

	return typePend;
}
#endif


/* @fn 	  instanceProcessRXTimeoutAnch
 * @brief function to process RX timeout event
 * */
void anch_process_RX_timeout(instance_data_t *inst)
{
    //inst->rxTimeouts ++ ;

    if(inst->mode == ANCHOR) //we did not receive the final - wait for next poll
    {
		//only enable receiver when not using double buffering
		inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
		dwt_setrxtimeout(0);
		inst->wait4ack = 0 ; //clear the flag,
    }
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
	else //ANCHOR_RNG
	{
		//no Response form the other anchors
        if((inst->testAppState == TA_TXPOLL_WAIT_SEND)
        		&& (inst->rxResponseMaskAnc == 0))
        {
        	//go back to RX and set ANCHOR mode
        	rnganch_change_back_to_anchor(inst);
        }
        else if (inst->previousState == TA_TXFINAL_WAIT_SEND) //got here from main (error ending final "late" - handle as timeout)
        {
        	//go back to RX and set ANCHOR mode
        	rnganch_change_back_to_anchor(inst);
        }
        else //send the final
        {
        	//initiate send the final
			inst->testAppState = TA_TXFINAL_WAIT_SEND ;
		}
	}
#endif
}


/**
 * @brief this function either enables the receiver (delayed)
 *
 **/
void anch_enable_rx(uint32 dlyTime)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);
	//subtract preamble duration (because when instructing delayed TX the time is the time of SFD,
	//however when doing delayed RX the time is RX on time)
	dwt_setdelayedtrxtime(dlyTime - inst->preambleDuration32h) ;
	if(dwt_rxenable(DWT_START_RX_DELAYED|DWT_IDLE_ON_DLY_ERR)) //delayed rx
	{
		//if the delayed RX failed - time has passed - do immediate enable
		dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy*2); //reconfigure the timeout before enable
		//longer timeout as we cannot do delayed receive... so receiver needs to stay on for longer
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy); //restore the timeout for next RX enable
		//inst->lateRX++;
	}
}

/**
 * @brief this function either re-enables the receiver (delayed or immediate) or transmits the response frame
 *
 * @param error - set to 1 if coming here from timeout or error callback
 *
 */
uint8 anch_txresponse_or_rx_reenable(void)
{
	uint8 typePend = DWT_SIG_DW_IDLE;
	int sendResp = 0;
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	//if remainingRespToRx == 0 - all the expected responses have been received (or timed out or errors instead)
	if(inst->remainingRespToRx == 0) //go back to RX without timeout - ranging has finished. (wait for Final but no timeout)
	{
		dwt_setrxtimeout(inst->fwto4FinalFrame_sy*2); //reconfigure the timeout for the final
		inst->wait4final = WAIT4TAGFINAL;
	}

	if((inst->remainingRespToRx + inst->shortAdd_idx) == NUM_EXPECTED_RESPONSES)
	{
		sendResp = 1;
	}

	//configure delayed reply time (this is incremented for each received frame) it is timed from Poll rx time
	inst->delayedTRXTime32h += inst->fixedReplyDelayAnc32h;

	//this checks if to send a response
	if(sendResp == 1)
	{
		//response is expected
		inst->wait4ack = DWT_RESPONSE_EXPECTED; //re has/will be re-enabled

		dwt_setdelayedtrxtime(inst->delayedTRXTime32h) ;
		if(dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED))
		{
			//if TX has failed - we need to re-enable RX for the next response or final reception...
			dwt_setrxaftertxdelay(0);
			inst->wait4ack = 0; //clear the flag as the TX has failed the TRX is off
			//inst->lateTX++;

			if(inst->remainingRespToRx == 0) //not expecting any more responses - enable RX
			{
				inst->delayedTRXTime32h = (inst->tagPollRxTime >> 8) + inst->pollTx2FinalTxDelay;
			}
			else
			{
				inst->delayedTRXTime32h += 2*(inst->fixedReplyDelayAnc32h); //to take into account W4R
			}
			anch_enable_rx(inst->delayedTRXTime32h);
			typePend = DWT_SIG_RX_PENDING ;
		}
		else
		{
			inst->delayedTRXTime32h += inst->fixedReplyDelayAnc32h; //to take into account W4R
			typePend = DWT_SIG_TX_PENDING ; // exit this interrupt and notify the application/instance that TX is in progress.
			inst->timeofTx = portGetTickCnt();
			inst->monitor = 1;
		}
	}
	else //stay in receive
	{
		if(inst->remainingRespToRx == 0) //not expecting any more responses - enable RX
		{
			inst->delayedTRXTime32h = (inst->tagPollRxTime >> 8) + (inst->pollTx2FinalTxDelay >> 8);
			dwt_setdelayedtrxtime(inst->delayedTRXTime32h - inst->preambleDuration32h) ;
			//dwt_setdelayedtrxtime(inst->delayedReplyTime);
			if(dwt_rxenable(DWT_START_RX_DELAYED|DWT_IDLE_ON_DLY_ERR)) //delayed rx
			{
				anch_no_timeout_rx_reenable();
			}
		}
		else
		{
			anch_enable_rx(inst->delayedTRXTime32h);
		}

		typePend = DWT_SIG_RX_PENDING ;
	}
	//if time to send a response
	return typePend;
}


/**
 * @brief this function prepares and writes the anchor to tag response frame into the TX buffer
 * it is called after anchor receives a Poll from a tag
 */
void anch_prepare_anc2tag_response(unsigned int tof_idx, uint8 srcAddr_index, uint8 fcode_index, uint8 *frame, uint32 uTimeStamp)
{
	uint16 frameLength = 0;
	instance_data_t* inst = instance_get_local_structure_ptr(0);
	int tagSleepCorrection_ms = 0;

	inst->psduLength = frameLength = ANCH_RESPONSE_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC;
	memcpy(&inst->msg_f.destAddr[0], &frame[srcAddr_index], ADDR_BYTE_SIZE_S); //remember who to send the reply to (set destination address)
	inst->msg_f.sourceAddr[0] = inst->eui64[0];
	inst->msg_f.sourceAddr[1] = inst->eui64[1];
	// Write calculated TOF into response message (get the previous ToF+range number from that tag)
	memcpy(&(inst->msg_f.messageData[TOFR]), &inst->tof[tof_idx], 4);
	inst->msg_f.messageData[TOFRN] = inst->rangeNumA[tof_idx]; //get the previous range number

	inst->rangeNumA[tof_idx] = 0; //clear after copy above...
	inst->rangeNum = frame[POLL_RNUM+fcode_index] ;
	inst->msg_f.seqNum = inst->frameSN++;

	//we have our range - update the own mask entry...
	if(inst->tof[tof_idx] != INVALID_TOF) //check the last ToF entry is valid and copy into the current array
	{
		inst->rxResponseMask = (0x1 << inst->shortAdd_idx);
		inst->tofArray[inst->shortAdd_idx] = inst->tof[tof_idx];
	}
	else	//reset response mask
	{
		inst->tofArray[inst->shortAdd_idx] = INVALID_TOF ;
		inst->rxResponseMask = 0;	//reset the mask of received responses when rx poll
	}
	//set the delayed rx on time (the final message will be sent after this delay)
	dwt_setrxaftertxdelay(inst->ancRespRxDelay_sy);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

	//if this is gateway anchor - calculate the slot period correction
	if(inst->gatewayAnchor)
	{
		int error = 0;
		int currentSlotTime = 0;
		int expectedSlotTime = 0;
		//find the time in the current superframe
		currentSlotTime = uTimeStamp % inst->sframePeriod_ms;

		//this is the slot time the poll should be received in (Mask 0x07 for the 8 MAX tags we support in TREK)
		expectedSlotTime = tof_idx * inst->slotDuration_ms; //

		//error = expectedSlotTime - currentSlotTime
		error = expectedSlotTime - currentSlotTime;

		if(error < (-(inst->sframePeriod_ms>>1))) //if error is more negative than 0.5 period, add whole period to give up to 1.5 period sleep
		{
			tagSleepCorrection_ms = (inst->sframePeriod_ms + error);
		}
		else //the minimum Sleep time will be 0.5 period
		{
			tagSleepCorrection_ms = error;
		}
		inst->msg_f.messageData[RES_TAG_SLP0] = tagSleepCorrection_ms & 0xFF ;
		inst->msg_f.messageData[RES_TAG_SLP1] = (tagSleepCorrection_ms >> 8) & 0xFF;
	}
	else
	{
		tagSleepCorrection_ms = 0;
		inst->msg_f.messageData[RES_TAG_SLP0] = 0 ;
		inst->msg_f.messageData[RES_TAG_SLP1] = 0 ;
	}
	inst->msg_f.messageData[FCODE] = RTLS_DEMO_MSG_ANCH_RESP; //message function code (specifies if message is a poll, response or other...)

	//write the TX data
	dwt_writetxfctrl(frameLength, 0, 1);
	dwt_writetxdata(frameLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data

}


/**
 * @brief this function handles frame error event, it will either signal timeout or re-enable the receiver
 */
void anch_handle_error_unknownframe_timeout(event_data_t dw_event)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);

	/*Note: anchor can be involved in anchor-to-anchor ranging ANCHOR_RNG mode or just ANCHOR mode (ranging with tags)
	 * In the former case if the anchor is initiating ranging exchange and after timeout/error
	 * it needs to send a final message or re-enable receiver to receiver more responses.
	 * In the latter case if the anchor get an error or timeout it may want to send response to the tag or re-enable the receiver.
	 */
	dw_event.type = 0;
	//dw_event.typeSave = 0x40 | DWT_SIG_RX_TIMEOUT;
	dw_event.rxLength = 0;

	//check if the anchor has received any of the responses from other anchors
	//it's timed out (re-enable rx or tx final)
	if(inst->remainingRespToRx >= 0) //if it was expecting responses - then decrement count
	{
		inst->remainingRespToRx--;
	}

	switch(inst->mode)
	{
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
		case ANCHOR_RNG:
		{
			//report timeout to application
			//e.g. A0 got error instead of response from A1 or A2
			// or A0 got timeout - if timeout need to check if RX any responses and then send final
			if(inst->remainingRespToRx == 0)
			{
                dw_event.typePend = DWT_SIG_DW_IDLE; //RX finished
				instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
			}
			else //got an error while waiting for A1's response (re-enable RX) - don't report any events
			{
				dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
			}
			break;
		}
#endif
		case ANCHOR:
		{
			//check firstly if involved in the ranging exchange with a tag
			if(inst->twrMode == RESPONDER_T)
			{
				if(inst->wait4final == WAIT4TAGFINAL)
				{
					inst->twrMode = LISTENER ;
					inst->wait4final = 0;

					dw_event.typePend = DWT_SIG_DW_IDLE;
					instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
				}
				else
				{
					dw_event.typePend = anch_txresponse_or_rx_reenable();

					//if rx error or timeout and then is sending response or in idle
					//report timeout to application
					if(dw_event.typePend != DWT_SIG_RX_PENDING)
					{
						instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
					}
				}
				break;
			}
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
			if(inst->twrMode == RESPONDER_A)
			{
			    if(inst->wait4final == WAIT4ANCFINAL)
			    {
					inst->twrMode = LISTENER ;
			    	inst->wait4final = 0;

			    	dw_event.typePend = DWT_SIG_DW_IDLE;
			    	instance_putevent(dw_event, DWT_SIG_RX_TIMEOUT);
			    	break;
			    }
			}
#endif
		}
		default: //e.g. listener - got error, and not participating in ranging - ignore event
		{
			dwt_rxenable(DWT_START_RX_IMMEDIATE) ;
		}
		break;
	}

	/*if(inst->remainingRespToRx == 0)
	{
		inst->remainingRespToRx = -1;
	}*/
}

/**
 * @brief this is the receive timeout event callback handler
 */

void rx_to_cb_anch(const dwt_cb_data_t *rxd)
{
	event_data_t dw_event;

	//microcontroller time at which we received the frame
    dw_event.uTimeStamp = portGetTickCnt();
   	anch_handle_error_unknownframe_timeout(dw_event);
}

/**
 * @brief this is the receive error event callback handler
 */
void rx_err_cb_anch(const dwt_cb_data_t *rxd)
{
	event_data_t dw_event;

	//microcontroller time at which we received the frame
    dw_event.uTimeStamp = portGetTickCnt();
    anch_handle_error_unknownframe_timeout(dw_event);

}

/**
 * @brief this is the receive event callback handler, the received event is processed and the instance either
 * responds by sending a response frame or re-enables the receiver to await the next frame
 * once the immediate action is taken care of the event is queued up for application to process
 */
void rx_ok_cb_anch(const dwt_cb_data_t *rxd)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);
	uint8 rxTimeStamp[5]  = {0, 0, 0, 0, 0};

    uint8 rxd_event = 0;
	uint8 fcode_index  = 0;
	uint8 srcAddr_index = 0;
	event_data_t dw_event;

	//microcontroller time at which we received the frame
    dw_event.uTimeStamp = portGetTickCnt();

    //if we got a frame with a good CRC - RX OK
    {
 		dw_event.rxLength = rxd->datalength;

		//need to process the frame control bytes to figure out what type of frame we have received
		if((rxd->fctrl[0] == 0x41) //no auto-ACK
				&&
				((rxd->fctrl[1] & 0xCC) == 0x88)) //short address
		{

			fcode_index = FRAME_CRTL_AND_ADDRESS_S; //function code is in first byte after source address
			srcAddr_index = FRAME_CTRLP + ADDR_BYTE_SIZE_S;
			rxd_event = DWT_SIG_RX_OKAY;
		}
#if (DISCOVERY == 1)
		else if (inst->gatewayAnchor && (rxd->fctrl[0] == 0xC5) && (inst->twrMode == LISTENER)) //only gateway anchor processes blinks
		{
			rxd_event = DWT_SIG_RX_BLINK;
		}
#endif
		else
		{
			rxd_event = SIG_RX_UNKNOWN; //not supported - all TREK1000 frames are short addressed
		}

        //read RX timestamp
        dwt_readrxtimestamp(rxTimeStamp) ;
        dwt_readrxdata((uint8 *)&dw_event.msgu.frame[0], rxd->datalength, 0);  // Read Data Frame
        instance_seteventtime(&dw_event, rxTimeStamp);

        dw_event.type = 0; //type will be added as part of adding to event queue
		//dw_event.typeSave = rxd_event;
		dw_event.typePend = DWT_SIG_DW_IDLE;
#if (DISCOVERY == 1)
		if(rxd_event == DWT_SIG_RX_BLINK)
		{
			int slot = anch_add_tag_to_list(&(dw_event.msgu.rxblinkmsg.tagID[0]));
			//add this Tag to the list of Tags we know about
			//if the returned value is not -1 then we want to add this tag (the returned value is the slot number)
			if(slot != -1)
			{
				inst->twrMode = RESPONDER_B ;
				inst->delayedTRXTime32h = dw_event.timeStamp32h + inst->fixedReplyDelayAnc32h;
				//send response (ranging init) with slot number / tag short address
				//prepare the response and write it to the tx buffer
				dw_event.typePend = anch_prepare_anc2tag_rangeinitresponse(&(dw_event.msgu.rxblinkmsg.tagID[0]), slot, dw_event.uTimeStamp);

				if(dw_event.typePend == DWT_SIG_TX_PENDING)
				{
					instance_putevent(dw_event, rxd_event);
					//inst->rxMsgCount++;
				}
				else
				{
					anch_handle_error_unknownframe_timeout(dw_event);
				}
			}
			else
			{
				//we cannot add this tag into our system...
				anch_handle_error_unknownframe_timeout(dw_event);
			}

		}
		else
#endif
		if(rxd_event == DWT_SIG_RX_OKAY) //Process good/known frame types
		{
			uint16 sourceAddress = (((uint16)dw_event.msgu.frame[srcAddr_index+1]) << 8) + dw_event.msgu.frame[srcAddr_index];
			//PAN ID must match - else discard this frame
			if((dw_event.msgu.rxmsg_ss.panID[0] != (inst->panID & 0xff)) ||
					(dw_event.msgu.rxmsg_ss.panID[1] != (inst->panID >> 8)))
			{
				anch_handle_error_unknownframe_timeout(dw_event);
				return;
			}
			//check if this is a TWR message (and also which one)
			switch(dw_event.msgu.frame[fcode_index])
			{
				//poll message from an anchor
				case RTLS_DEMO_MSG_ANCH_POLL:
				{
					//here we should check if this poll message is for us, i.e. if we are going to respond
					//for TREK in the anchor to anchor ranging the poll frames are ignored by A0 and A3
					//as the anchor to anchor ranging is fixed (A0 ranges to A1, A2; A1 ranges to A2)
					//group poll could have a list of anchors that need to reply
					//if not for us handle as timeout / error
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
					if(inst->gatewayAnchor || (inst->instanceAddress16 > A2_ANCHOR_ADDR))
#endif
					{
						anch_no_timeout_rx_reenable();
						return;
					}
#if (ANCTOANCTWR == 1) //anchor to anchor ranging

					inst->twrMode = RESPONDER_A ;
					inst->wait4final = 0;

					//prepare the response and write it to the tx buffer
					anch_prepare_anc2anc_response(sourceAddress, srcAddr_index, fcode_index, &dw_event.msgu.frame[0]);

					//if 1st anchor in the sequence then send reply otherwise schedule TX in future
					//during anchor to anchor ranging A0 sends Poll, A1 ans A2 Respond and then A0 sends final
					//afterwards A1 ranges to A2, during Anc to Anc ranging A2 will not be waiting for A1's response (to A0)
					dw_event.typePend = rnganch_txresponse_or_rx_reenable(sourceAddress, dw_event.timeStamp32h);

					inst->tofAnc[sourceAddress & 0x3] = INVALID_TOF; //clear ToF ..

#endif
					break;
				}

				case RTLS_DEMO_MSG_TAG_POLL:
				{
					//if ANCHOR_RNG ignore tag polls
					//source address is used as index so has to be < MAX_TAG_LIST_SIZE or frame is ignored
					if((inst->mode == ANCHOR) && (sourceAddress < MAX_TAG_LIST_SIZE))
					{
						inst->twrMode = RESPONDER_T ;
						inst->wait4final = 0;

						//prepare the response and write it to the tx buffer
						anch_prepare_anc2tag_response(sourceAddress, srcAddr_index, fcode_index, &dw_event.msgu.frame[0], dw_event.uTimeStamp);

						dwt_setrxtimeout((uint16)inst->fwto4RespFrame_sy); //reconfigure the timeout for response

						inst->delayedTRXTime32h = dw_event.timeStamp32h ;
						inst->remainingRespToRx = NUM_EXPECTED_RESPONSES; //set number of expected responses to 3 (from other anchors)

						dw_event.typePend = anch_txresponse_or_rx_reenable();

						inst->tof[sourceAddress] = INVALID_TOF; //clear ToF ..

					}
					else //not participating in this exchange - ignore this frame
					{
						anch_handle_error_unknownframe_timeout(dw_event);
						return;
					}
				}
				break;

				//we got a response from a "responder" (anchor), involved in tag-to-anchor ranging
				case RTLS_DEMO_MSG_ANCH_RESP:
				{
					if(inst->twrMode == RESPONDER_T)
					{
						// are participating in this TWR exchange - need to check if time to send response or go back to RX
						// got a response... (check if we got a Poll with the same range number as in this response)
						inst->rxResps++; //increment the number of responses received
						inst->remainingRespToRx--; //reduce number of expected responses (as we have just received one)

						//send a response or re-enable rx
						dw_event.typePend = anch_txresponse_or_rx_reenable();
					}
					else //not participating in this exchange - ignore this frame
					{
						anch_no_timeout_rx_reenable();
						return;
					}
				}
				break;

				//we got a response from a "responder" (anchor), involved in anchor-to-anchor ranging
				case RTLS_DEMO_MSG_ANCH_RESP2:
				{
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
					//if we are the initiator
					if (inst->mode == ANCHOR_RNG) //A0 and A1 only when ranging to other anchors
					{
						uint8 index ;
						inst->rxResps++;
						inst->remainingRespToRx--;
						//check if this is the last expected response and need to send final next
						dw_event.typePend = rnganchrxresp_signalsendfinal_or_rx_reenable();

						//claculate the index in the Final message where the response time goes
						index = RRXT0 + 5*(sourceAddress & 0x3);
						inst->rxResponseMaskAnc |= (0x1 << (sourceAddress & 0x3)); //add anchor ID to the mask
						// Write Response RX time field of Final message
						memcpy(&(inst->msg_f.messageData[index]), rxTimeStamp, 5);
					}
					else //ANCHOR mode
					{

						//not participating in this exchange
#endif
						{
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
							// for anchor to anchor ranging A0 needs to process A2's response to A1's poll to
							// receive the A1-A2 TOF result and report it
							if((inst->gatewayAnchor) &&
								(inst->rxResps == 2))
							{//got two responses A1 and A2 this is third (A2's to A1)
								inst->rxResps++;
								inst->remainingRespToRx--;
								inst->rxResponseMaskAnc |= 0x8 ;

								anch_no_timeout_rx_reenable(); //stay in receive (wait for any tag polls)
								dw_event.typePend = DWT_SIG_RX_PENDING ;
							}
							else //ignore this frame
#endif
							{
								anch_no_timeout_rx_reenable();
								return;
							}
						}
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
					}
#endif

				}
				break;

				case RTLS_DEMO_MSG_TAG_FINAL:
					if((inst->twrMode == RESPONDER_T)&&(inst->mode == ANCHOR))
					{
						anch_no_timeout_rx_reenable();  // turn RX on, without delay
						dw_event.typePend = DWT_SIG_RX_PENDING ;
						inst->twrMode = LISTENER ;
						inst->wait4final = 0;
						break;
					}
				case RTLS_DEMO_MSG_ANCH_FINAL:
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
					if((inst->twrMode == RESPONDER_A)&&(inst->mode == ANCHOR))
					{
						anch_no_timeout_rx_reenable();  // turn RX on, without delay
						dw_event.typePend = DWT_SIG_RX_PENDING ;
						inst->twrMode = LISTENER ;
						inst->wait4final = 0;
						break;
					}
#endif
				default:  //ignore this frame
				{
					anch_no_timeout_rx_reenable();
					return;
				}
				break;

			}

            instance_putevent(dw_event, rxd_event);

			//inst->rxMsgCount++;
		}
		else //if (rxd_event == SIG_RX_UNKNOWN) //need to re-enable the rx (got unknown frame type)
		{
			anch_handle_error_unknownframe_timeout(dw_event);
		}
	}
}
// -------------------------------------------------------------------------------------------------------------------
//
// the main instance state machine for anchor application
//
// -------------------------------------------------------------------------------------------------------------------
//

int check_state_change(uint8 event)
{
	int state = TA_RXE_WAIT ;
	//the response has been sent - await TX done event
	if(event == DWT_SIG_TX_PENDING)
	{
		state = TA_TX_WAIT_CONF;                // wait confirmation
	}
	//already re-enabled the receiver
	else if (event == DWT_SIG_RX_PENDING)
	{
		//stay in RX wait for next frame...
		//RX is already enabled...
		state = TA_RX_WAIT_DATA ;              // wait for next frame
	}
	else //the DW1000 is idle (re-enable from the application level)
	{
		//stay in RX change state and re-enable RX for next frame...
		state = TA_RXE_WAIT ;
	}
	return state ;
}

int32 calc_tof(uint8 *messageData, uint64 anchorRespTxTime, uint64 tagFinalRxTime, uint64 tagPollRxTime, uint8 shortAdd_idx)
{
	int64 Rb, Da, Ra, Db ;
	uint64 tagFinalTxTime  = 0;
	uint64 tagPollTxTime  = 0;
	uint64 anchorRespRxTime  = 0;

	double RaRbxDaDb = 0;
	double RbyDb = 0;
	double RayDa = 0;
	int32 tof;

	uint8 index = RRXT0 + 5*(shortAdd_idx);

	// times measured at Tag extracted from the message buffer
	// extract 40bit times
	memcpy(&tagPollTxTime, &(messageData[PTXT]), 5);
	memcpy(&anchorRespRxTime, &(messageData[index]), 5);
	memcpy(&tagFinalTxTime, &(messageData[FTXT]), 5);

	// poll response round trip delay time is calculated as
	// (anchorRespRxTime - tagPollTxTime) - (anchorRespTxTime - tagPollRxTime)
	Ra = (int64)((anchorRespRxTime - tagPollTxTime) & MASK_40BIT);
	Db = (int64)((anchorRespTxTime - tagPollRxTime) & MASK_40BIT);

	// response final round trip delay time is calculated as
	// (tagFinalRxTime - anchorRespTxTime) - (tagFinalTxTime - anchorRespRxTime)
	Rb = (int64)((tagFinalRxTime - anchorRespTxTime) & MASK_40BIT);
	Da = (int64)((tagFinalTxTime - anchorRespRxTime) & MASK_40BIT);

	RaRbxDaDb = (((double)Ra))*(((double)Rb))
	- (((double)Da))*(((double)Db));

	RbyDb = ((double)Rb + (double)Db);

	RayDa = ((double)Ra + (double)Da);

	tof = (int32) ( RaRbxDaDb/(RbyDb + RayDa) );

	return tof;
}

int anch_app_run(instance_data_t *inst)
{
	int instDone = INST_NOT_DONE_YET;
    int message = instance_peekevent(); //get any of the received events from ISR

    switch (inst->testAppState)
    {
         case TA_INIT :
            // printf("TA_INIT") ;
            switch (inst->mode)
            {
                case ANCHOR:
                {
                    memcpy(inst->eui64, &inst->instanceAddress16, ADDR_BYTE_SIZE_S);
                    dwt_seteui(inst->eui64);

                    dwt_setpanid(inst->panID);

                    //set source address
                    inst->shortAdd_idx = (inst->instanceAddress16 & 0x3) ;
                    dwt_setaddress16(inst->instanceAddress16);

                	//if address = 0x8000
                	if(inst->instanceAddress16 == GATEWAY_ANCHOR_ADDR)
                	{
                		inst->gatewayAnchor = TRUE;
                	}

                	dwt_enableframefilter(DWT_FF_NOTYPE_EN); //allow data, ack frames;

                	// First time anchor listens we don't do a delayed RX
					dwt_setrxaftertxdelay(0);
                    //change to next state - wait to receive a message
                    inst->testAppState = TA_RXE_WAIT ;

                    dwt_setrxtimeout(0);
                    instance_config_frameheader_16bit(inst);

					//set frame type (0-2), SEC (3), Pending (4), ACK (5), PanIDcomp(6)
					inst->rng_initmsg.frameCtrl[0] = 0x1 /*frame type 0x1 == data*/ | 0x40 /*PID comp*/;

					//source/dest addressing modes and frame version
					inst->rng_initmsg.frameCtrl[1] = 0xC /*dest extended address (64bits)*/ | 0x80 /*src short address (16bits)*/;

					inst->rng_initmsg.panID[0] = (inst->panID) & 0xff;
					inst->rng_initmsg.panID[1] = inst->panID >> 8;


#if (ANCTOANCTWR == 1) //allow anchor to anchor ranging
					if(inst->gatewayAnchor)
					{
						uint32_t nowTime = portGetTickCnt();
						uint32_t slotTime = portGetTickCnt() % inst->sframePeriod_ms;

						//start of SF = nowTime - slotTime
						//start of A2A ranging =
						inst->a2aStartTime_ms = nowTime - slotTime + inst->a0SlotTime_ms ;
						//skip 5 first frames
						inst->a2aStartTime_ms = inst->a2aStartTime_ms + 5*inst->sframePeriod_ms ;

					}
#endif
                }
                break;
                default:
                break;
            }
            break; // end case TA_INIT
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
         case TA_TXPOLL_WAIT_SEND :
            {
            	/*Note: In TREK demo, last two slots of the superframe are used for anchor to anchor ranging,
            	 * A0 initiates ranging, and it will range to 2 anchors A1 and A2. This is sufficient for TREK demo
            	 * system. If A0 needed to range to other anchors, their addresses could be added to the poll message
            	 * so the anchors would know to respond.
            	 */

                inst->msg_f.messageData[POLL_RNUM] = inst->rangeNumAnc; //copy new range number
            	inst->msg_f.messageData[FCODE] = RTLS_DEMO_MSG_ANCH_POLL; //message function code (specifies if message is a poll, response or other...)

            	if(inst->gatewayAnchor)
            	{
            		inst->msg_f.messageData[POLL_NANC] = A1_ANCHOR_ADDR & 0xff;
            		inst->msg_f.messageData[POLL_NANC+1] = (A1_ANCHOR_ADDR >> 8) & 0xff;
            		inst->psduLength = (ANCH_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
            	}
            	else
            	{
            		inst->psduLength = (ANCH_POLL_MSG_LEN_S + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
            	}
                inst->msg_f.seqNum = inst->frameSN++; //copy sequence number and then increment
                inst->msg_f.sourceAddr[0] = inst->eui64[0]; //copy the address
                inst->msg_f.sourceAddr[1] = inst->eui64[1]; //copy the address
            	inst->msg_f.destAddr[0] = 0xff;  //set the destination address (broadcast == 0xffff)
            	inst->msg_f.destAddr[1] = 0xff;  //set the destination address (broadcast == 0xffff)
                dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data

				//set the delayed rx on time (the response message will be sent after this delay (from A0))
				dwt_setrxaftertxdelay((uint32)RX_RESPONSE_TURNAROUND);  //units are 1.0256us - wait for wait4respTIM before RX on (delay RX)

				dwt_setrxtimeout((uint16)inst->fwtoTime_sy);  //units are

				inst->rxResps = 0;
				inst->rxResponseMask = 0;	//reset/clear the mask of received responses when tx poll
				inst->rxResponseMaskAnc = 0;

				inst->wait4ack = DWT_RESPONSE_EXPECTED; //response is expected - automatically enable the receiver

				dwt_writetxfctrl(inst->psduLength, 0, 1); //write frame control

				dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED); //transmit the frame

                inst->testAppState = TA_TX_WAIT_CONF ;  // wait confirmation
                inst->previousState = TA_TXPOLL_WAIT_SEND ;
                instDone = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set above)
            }
            break;

        case TA_TXFINAL_WAIT_SEND :
            {
            	//the final has the same range number as the poll (part of the same ranging exchange)
                inst->msg_f.messageData[POLL_RNUM] = inst->rangeNumAnc;
                //the mask is sent so the anchors know whether the response RX time is valid
				inst->msg_f.messageData[VRESP] =  inst->rxResponseMaskAnc;
            	inst->msg_f.messageData[FCODE] = RTLS_DEMO_MSG_ANCH_FINAL; //message function code (specifies if message is a poll, response or other...)
                inst->psduLength = (ANCH_FINAL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC);
                inst->msg_f.seqNum = inst->frameSN++;
				dwt_writetxdata(inst->psduLength, (uint8 *)  &inst->msg_f, 0) ;	// write the frame data

				inst->wait4ack = 0; //clear the flag not using wait for response as this message ends the ranging exchange

				if(instance_send_delayed_frame(inst, DWT_START_TX_DELAYED))
                {
                    //anchor doing TWR to another anchor
					//A0 - failed to send Final
					//A1 - failed to send Final
					//go back to RX and set ANCHOR mode
					rnganch_change_back_to_anchor(inst);
                    break; //exit this switch case...
                }
                else
                {

                    inst->testAppState = TA_TX_WAIT_CONF;                                               // wait confirmation
                    inst->previousState = TA_TXFINAL_WAIT_SEND;

                }

				instDone = INST_DONE_WAIT_FOR_NEXT_EVENT; //will use RX FWTO to time out (set above)
            }
            break;
#endif

        case TA_TX_WAIT_CONF :
                {
				event_data_t* dw_event = instance_getevent(11); //get and clear this event

                if(dw_event->type != DWT_SIG_TX_DONE) //wait for TX done confirmation
                {
				    instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                    break;
                }

                instDone = INST_NOT_DONE_YET;

#if (ANCTOANCTWR == 1) //anchor to anchor ranging
                if(inst->previousState == TA_TXFINAL_WAIT_SEND)
                {
					//go back to RX and set ANCHOR mode
                	rnganch_change_back_to_anchor(inst);
                }
                else
#endif
                {
					inst->txu.txTimeStamp = dw_event->timeStamp;
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
					if(inst->previousState == TA_TXPOLL_WAIT_SEND)
					{
		                uint64 tagCalculatedFinalTxTime ;
		                // Embed into Final message: 40-bit pollTXTime,  40-bit respRxTime,  40-bit finalTxTime
		                //for anchor make the final half the delay ..... (this is ok, as A0 awaits 2 responses)
		                {
		                	tagCalculatedFinalTxTime =  (inst->txu.txTimeStamp + inst->pollTx2FinalTxDelayAnc) & MASK_TXDTS;
		                }
		                inst->delayedTRXTime32h = tagCalculatedFinalTxTime >> 8; //high 32-bits
		                // Calculate Time Final message will be sent and write this field of Final message
		                // Sending time will be delayedReplyTime, snapped to ~125MHz or ~250MHz boundary by
		                // zeroing its low 9 bits, and then having the TX antenna delay added
		                // getting antenna delay from the device and add it to the Calculated TX Time
		                tagCalculatedFinalTxTime = tagCalculatedFinalTxTime + inst->txAntennaDelay;
		                tagCalculatedFinalTxTime &= MASK_40BIT;

		                // Write Calculated TX time field of Final message
						memcpy(&(inst->msg_f.messageData[FTXT]), (uint8 *)&tagCalculatedFinalTxTime, 5);
		                // Write Poll TX time field of Final message
						memcpy(&(inst->msg_f.messageData[PTXT]), (uint8 *)&inst->txu.tagPollTxTime, 5);

					}
#endif
		            if(inst->previousState == TA_TXRESPONSE_SENT_TORX)
		            {
		            	inst->previousState = TA_TXRESPONSE_WAIT_SEND ;
		            }
                    inst->testAppState = TA_RXE_WAIT ;                      // After sending, tag expects response/report, anchor waits to receive a final/new poll

                    message = 0;
                    //fall into the next case (turn on the RX)
                }

            }

            //break ; // end case TA_TX_WAIT_CONF


        case TA_RXE_WAIT :
        // printf("TA_RXE_WAIT") ;
        {

            if(inst->wait4ack == 0) //if this is set the RX will turn on automatically after TX
            {
            	if(dwt_read16bitoffsetreg(0x19,1) != 0x0505)
            	{
            		//turn RX on
            	  	dwt_rxenable(DWT_START_RX_IMMEDIATE) ;  // turn RX on, without delay
            	}
            }
            else
            {
                inst->wait4ack = 0 ; //clear the flag, the next time we want to turn the RX on it might not be auto
            }

            instDone = INST_DONE_WAIT_FOR_NEXT_EVENT; //using RX FWTO
            inst->testAppState = TA_RX_WAIT_DATA;   // let this state handle it

            // end case TA_RXE_WAIT, don't break, but fall through into the TA_RX_WAIT_DATA state to process it immediately.
            if(message == 0) break;
        }

        case TA_RX_WAIT_DATA :                                                                     // Wait RX data
		   //printf("TA_RX_WAIT_DATA %d", message) ;

            switch (message)
            {

				//if we have received a DWT_SIG_RX_OKAY event - this means that the message is IEEE data type - need to check frame control to know which addressing mode is used
                case DWT_SIG_RX_OKAY :
                {
					event_data_t* dw_event = instance_getevent(15); //get and clear this event
					uint8  srcAddr[8] = {0,0,0,0,0,0,0,0};
					uint8  dstAddr[8] = {0,0,0,0,0,0,0,0};
                    int fcode = 0;
					uint8 tof_idx  = 0;
					int tag_index = 0;
					uint8 *messageData;

					memcpy(&srcAddr[0], &(dw_event->msgu.rxmsg_ss.sourceAddr[0]), ADDR_BYTE_SIZE_S);
					memcpy(&dstAddr[0], &(dw_event->msgu.rxmsg_ss.destAddr[0]), ADDR_BYTE_SIZE_S);
					fcode = dw_event->msgu.rxmsg_ss.messageData[FCODE];
					messageData = &dw_event->msgu.rxmsg_ss.messageData[0];

					tof_idx = srcAddr[0] & 0x3 ;
					//process ranging messages
					switch(fcode)
					{
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
						case RTLS_DEMO_MSG_ANCH_POLL:
							inst->rangeNumAAnc[tof_idx] = messageData[POLL_RNUM]; //when anchor receives poll from another anchor - save the range number

							//check if this poll message contains the next anchor to range
							if(dw_event->rxLength == (ANCH_POLL_MSG_LEN + FRAME_CRTL_AND_ADDRESS_S + FRAME_CRC))
							{
								uint16 nextAnchorToRange = messageData[POLL_NANC] + (((uint16)messageData[POLL_NANC+1]) << 8);

								//if the next anchor to range is us, configure the ranging slot time
								if (nextAnchorToRange == inst->instanceAddress16) //this is A1
								{
									if(GATEWAY_ANCHOR_ADDR == (srcAddr[0] | ((uint32)(srcAddr[1] << 8)))) //poll is from A0
									{
										//configure the time A1 will poll A2 (it should be in time next slot)
										inst->a1SlotTime_ms = dw_event->uTimeStamp + (inst->slotDuration_ms);

										//inst->instanceTimerEn = 1; - THIS IS ENABLED BELOW AFTER RECEPTION OF A0's FINAL
										// - means that if final is not received then A1 will not range to A2
									}
								}
							}

							inst->tagPollRxTime = dw_event->timeStamp ; //save Poll's Rx time

							//check if need to change state
							inst->testAppState = check_state_change(dw_event->typePend) ;
							if(inst->testAppState == TA_TX_WAIT_CONF)
							{
								inst->previousState = TA_TXRESPONSE_SENT_APOLLRX ;    //wait for TX confirmation of sent response
							}
						break;
#endif
						case RTLS_DEMO_MSG_TAG_POLL:
						{
							//source address is used as index so has to be < MAX_TAG_LIST_SIZE or frame is ignored
							//this check is done in the callback so the event here will have good address range
							tag_index = srcAddr[0] + (((uint16) srcAddr[1]) << 8);
							inst->rangeNumA[tag_index] = messageData[POLL_RNUM]; //when anchor receives a poll, we need to remember the new range number

							inst->tagPollRxTime = dw_event->timeStamp ; //save Poll's Rx time

							//check if need to change state
							inst->testAppState = check_state_change(dw_event->typePend) ;
							if(inst->testAppState == TA_TX_WAIT_CONF)
							{
								inst->previousState = TA_TXRESPONSE_SENT_POLLRX ;    //wait for TX confirmation of sent response
							}

						}
						break; //RTLS_DEMO_MSG_TAG_POLL
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
						case RTLS_DEMO_MSG_ANCH_RESP2:
						{
							uint8 currentRangeNum = (messageData[TOFRN] + 1); //current = previous + 1

							//the response has been sent - await TX done event
							if(dw_event->typePend == DWT_SIG_TX_PENDING) //anchor received response from anchor ID - 1 so is sending it's response now back to tag
							{
								inst->testAppState = TA_TX_WAIT_CONF;                // wait confirmation
								inst->previousState = TA_TXRESPONSE_SENT_ARESPRX ;    //wait for TX confirmation of sent response
							}
							//already re-enabled the receiver
							else if(dw_event->typePend == DWT_SIG_RX_PENDING)
							{
								// stay in TA_RX_WAIT_DATA - receiver is already enabled.
							}
							//DW1000 idle - send the final
							else
							{
								//if this is A0 and A1's or A2's response received
								if(inst->rxResponseMaskAnc != 0) // or if this is A1 and A2's response received
								{
									inst->testAppState = TA_TXFINAL_WAIT_SEND ; // send our response / the final
								}
								else //go back to ANCHOR mode (LISTENER)
								{
									//go back to RX and set ANCHOR mode
									rnganch_change_back_to_anchor(inst);
								}
							}

							//anchor to anchor (only gateway processes anchor to anchor ToFs)
							{
								//report the correct set of ranges (ranges from anchors A1, A2 need to match owns range number)
								if((inst->gatewayAnchor)&&(currentRangeNum == inst->rangeNumAnc)) //these are the previous ranges...
								{
									inst->rangeNumAAnc[0] = inst->rangeNumAnc ;

									//once A0 receives A2's response to A1's poll, then it can report the 3 ToFs.
									//so number of responses is 3: A1, A2 to A0's poll and A2 to A1's poll
									if(inst->rxResps == 3)
									//if(A2_ANCHOR_ADDR == (srcAddr[0] | ((uint32)(srcAddr[1] << 8))))
									{
										//copy the ToF and put into array, the array should have 3 ToFs A0-A1, A0-A2 and A1-A2
										memcpy(&inst->tofArrayAnc[(srcAddr[0]+dstAddr[0])&0x3], &(messageData[TOFR]), 4);
										//calculate all anchor - anchor ranges... and report
										inst->newRange = instance_calc_ranges(&inst->tofArrayAnc[0], MAX_ANCHOR_LIST_SIZE, TOF_REPORT_A2A, &inst->rxResponseMaskAnc);
										inst->rxResponseMaskReport = inst->rxResponseMaskAnc;
										inst->newRangeTime = dw_event->uTimeStamp ;
#if (READ_EVENT_COUNTERS == 1)
										dwt_readeventcounters(&inst->ecounters);
#endif
									}
									else
									{
										//copy the ToF and put into array (array holds last 4 ToFs)
										memcpy(&inst->tofArrayAnc[(srcAddr[0]+dstAddr[0])&0x3], &(messageData[TOFR]), 4);
									}
								}
							}

						}
						break;  //RTLS_DEMO_MSG_ANCH_RESP2
#endif
						case RTLS_DEMO_MSG_ANCH_RESP:
						{
							uint8 currentRangeNum = (messageData[TOFRN] + 1); //current = previous + 1

							//the response has been sent - await TX done event
							if(dw_event->typePend == DWT_SIG_TX_PENDING) //anchor received response from anchor ID - 1 so is sending it's response now back to tag
							{
								inst->testAppState = TA_TX_WAIT_CONF;                // wait confirmation
								inst->previousState = TA_TXRESPONSE_SENT_RESPRX ;    //wait for TX confirmation of sent response
							}
							//already re-enabled the receiver
							else if(dw_event->typePend == DWT_SIG_RX_PENDING)
							{
								// stay in TA_RX_WAIT_DATA - receiver is already enabled.
							}
							//DW1000 idle re-enable RX
							else
							{
								inst->testAppState = TA_RXE_WAIT ; // wait for next frame
							}

							if(currentRangeNum == inst->rangeNum) //these are the previous ranges...
							{
								//copy the ToF and put into array (array holds last 4 ToFs)
								memcpy(&inst->tofArray[tof_idx], &(messageData[TOFR]), 4);

								//check if the ToF is valid, this makes sure we only report valid ToFs
								//e.g. consider the case of reception of response from anchor a1 (we are anchor a2)
								//if a1 got a Poll with previous Range number but got no Final, then the response will have
								//the correct range number but the range will be INVALID_TOF
								if(inst->tofArray[tof_idx] != INVALID_TOF)
								{
									inst->rxResponseMask |= (0x1 << (tof_idx));
								}
							}
							else //mark as invalid (clear the array)
							{
								if(inst->tofArray[tof_idx] != INVALID_TOF)
								{
									inst->tofArray[tof_idx] = INVALID_TOF;
								}
							}

						}
						break; //RTLS_DEMO_MSG_ANCH_RESP
#if (ANCTOANCTWR == 1) //anchor to anchor ranging
						case RTLS_DEMO_MSG_ANCH_FINAL:
						{
							uint64 tof = INVALID_TOF;
							uint8 validResp = messageData[VRESP];

							if((((inst->rangeNumAAnc[tof_idx] != messageData[POLL_RNUM]) //Final's range number needs to match Poll's or else discard this message
									|| inst->gatewayAnchor) //gateway can ignore the Final (from A1 to A2 exchange)
									|| (A3_ANCHOR_ADDR == inst->instanceAddress16))) //A3 does not care about Final from A1 or A0
							{
								inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
								break;
							}

							if (A1_ANCHOR_ADDR == inst->instanceAddress16) //this is A1
							{
								if(GATEWAY_ANCHOR_ADDR == (srcAddr[0] | ((uint32)(srcAddr[1] << 8)))) //final is from A0
								{
									//ENABLE TIMER ONLY IF FINAL RECEIVED
									inst->instanceTimerEn = 1;
								}
							}

							//if we got the final, maybe the tag did not get our response, so
							//we can use other anchors responses/ToF if there are any.. and output..
							//but we cannot calculate new range
							if(((validResp & (0x1<<(inst->shortAdd_idx))) != 0))
							{
								tof = calc_tof(messageData, inst->txu.anchorRespTxTime, dw_event->timeStamp, inst->tagPollRxTime, inst->shortAdd_idx);
							}

							//anchor to anchor ranging
							inst->delayedTRXTime32h = 0 ;
							//time-of-flight
							inst->tofAnc[tof_idx] = tof;

#if (READ_EVENT_COUNTERS == 1)
							dwt_readeventcounters(&inst->ecounters);
#endif
							if(dw_event->typePend != DWT_SIG_RX_PENDING)
							{
								inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
							}
							//else stay in TA_RX_WAIT_DATA - receiver is already enabled.

						}
						break; //RTLS_DEMO_MSG_ANCH_FINAL
#endif
						case RTLS_DEMO_MSG_TAG_FINAL:
						{
							uint64 tof = INVALID_TOF;
							uint8 validResp = messageData[VRESP];
							//source address is used as index so has to be < MAX_TAG_LIST_SIZE or frame is ignored
							//this check is done in the callback so the event here will have good address range
							tag_index = srcAddr[0] + (((uint16) srcAddr[1]) << 8);

							if(inst->rangeNumA[tag_index] != messageData[POLL_RNUM]) //Final's range number needs to match Poll's or else discard this message
							{
								inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
								break;
							}

							//if we got the final, maybe the tag did not get our response, so
							//we can use other anchors responses/ToF if there are any.. and output..
							//but we cannot calculate new range
							if(((validResp & (0x1<<(inst->shortAdd_idx))) != 0))
							{
								tof = calc_tof(messageData, inst->txu.anchorRespTxTime, dw_event->timeStamp, inst->tagPollRxTime, inst->shortAdd_idx);
							}

							//tag to anchor ranging
							inst->newRangeAncAddress = inst->instanceAddress16;
							inst->delayedTRXTime32h = 0 ;
							inst->newRangeTagAddress = tag_index ;
							//time-of-flight
							inst->tof[tag_index] = tof;
							//calculate all tag - anchor ranges... and report
							inst->newRange = instance_calc_ranges(&inst->tofArray[0], MAX_ANCHOR_LIST_SIZE, TOF_REPORT_T2A, &inst->rxResponseMask);
							inst->rxResponseMaskReport = inst->rxResponseMask; //copy the valid mask to report
							inst->rxResponseMask = 0;
							//we have our range - update the own mask entry...
							if(tof != INVALID_TOF) //check the last ToF entry is valid and copy into the current array
							{
								instance_set_tagdist(tag_index, inst->shortAdd_idx); //copy distance from this anchor to the tag into array

								inst->rxResponseMask = (0x1 << inst->shortAdd_idx);
								inst->tofArray[inst->shortAdd_idx] = tof;
							}
							inst->newRangeTime = dw_event->uTimeStamp ;

							instance_set_antennadelays(); //this will update the antenna delay if it has changed
							instance_set_txpower(); // configure TX power if it has changed

#if (READ_EVENT_COUNTERS == 1)
							dwt_readeventcounters(&inst->ecounters);
#endif
							if(dw_event->typePend != DWT_SIG_RX_PENDING)
							{
								inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
							}
							//else stay in TA_RX_WAIT_DATA - receiver is already enabled.

						}
						break; //RTLS_DEMO_MSG_TAG_FINAL

						default:
						{
							//only enable receiver when not using double buffering
							inst->testAppState = TA_RXE_WAIT ;              // wait for next frame
							dwt_setrxaftertxdelay(0);

						}
						break;
					} //end switch (fcode)
                }
				break ; //end of DWT_SIG_RX_OKAY

                case DWT_SIG_RX_TIMEOUT :
                	{
                		event_data_t* dw_event = instance_getevent(17); //get and clear this event

                		//Anchor can time out and then need to send response - so will be in TX pending
                		if(dw_event->typePend == DWT_SIG_TX_PENDING)
                		{
                			inst->testAppState = TA_TX_WAIT_CONF;              // wait confirmation
                			inst->previousState = TA_TXRESPONSE_SENT_TORX ;    // wait for TX confirmation of sent response
                		}
                		else if(dw_event->typePend == DWT_SIG_DW_IDLE) //if timed out and back in receive then don't process as timeout
						{
                			anch_process_RX_timeout(inst);
                			instDone = INST_NOT_DONE_YET;
						}
                		//else if RX_PENDING then wait for next RX event...
						message = 0; //clear the message as we have processed the event
                	}
                break ;

				default :
                {
                    if(message)
                    {
                    	instance_getevent(20); //get and clear this event
                    }

                	if(instDone == INST_NOT_DONE_YET) instDone = INST_DONE_WAIT_FOR_NEXT_EVENT;
                }
                break;

            }
            break ; // end case TA_RX_WAIT_DATA
            default:
            break;
    } // end switch on testAppState

    return instDone;
} // end testapprun_anch()

// -------------------------------------------------------------------------------------------------------------------
int anch_run(void)
{
	instance_data_t* inst = instance_get_local_structure_ptr(0);
    int done = INST_NOT_DONE_YET;

	while(done == INST_NOT_DONE_YET)
	{
		done = anch_app_run(inst) ; // run the communications application
	}

#if (ANCTOANCTWR == 1) //allow anchor to anchor ranging
	if(inst->gatewayAnchor)
	{
		//if we are in the last slot - then A0 ranges to A1 and A2
		//if( slotTime >= inst->a0SlotTime)
		if(portGetTickCnt() >= inst->a2aStartTime_ms)
		{
			inst->a2aStartTime_ms += inst->sframePeriod_ms;

			//if not already involved in ranging exchange with a tag
			if((inst->mode == ANCHOR) && (inst->twrMode == LISTENER))
			{
				anch_change_to_rnganchor(inst);
				inst->msg_f.destAddr[0] = 0x1 ;
				inst->msg_f.destAddr[1] = (GATEWAY_ANCHOR_ADDR >> 8);
			}
			//else skip until next time .... anchor to anchor ranging is lower priority
		}
	}

	if ((inst->instanceAddress16 == A1_ANCHOR_ADDR) && (inst->instanceTimerEn > 0))//A1 ranges to A2 in the 2nd half of last slot
	{
		if(portGetTickCnt() >= inst->a1SlotTime_ms)
		{
			//if not already involved in ranging exchange with a tag
			if((inst->mode == ANCHOR) && (inst->twrMode == LISTENER))
			{
				anch_change_to_rnganchor(inst);
				inst->msg_f.destAddr[0] = 0x2 ;
				inst->msg_f.destAddr[1] = (GATEWAY_ANCHOR_ADDR >> 8);
			}
			//else skip until next time .... anchor to anchor ranging is lower priority

			inst->instanceTimerEn = 0;
		}
	}
#endif

    return 0 ;
}
/* ==========================================================

Notes:

Previously code handled multiple instances in a single console application

Now have changed it to do a single instance only. With minimal code changes...(i.e. kept [instance] index but it is always 0.

Windows application should call instance_init() once and then in the "main loop" call instance_run().

*/
