/*
 * mac_fragmentation.c
 *
 *  Created on: Jan 13, 2020
 *      Author: lukas
 */

#include "mac_fragmentation.h"

#include "../util/ringbuf.h"
#include "mac_config.h"

#define MAX_SEQNR 4 // 2 bits are allocated for seqNr in MacMessage
#define MAX_FRAGNR 32 // 5 bits are allocated for fragNr in MacMessage


struct MacFragmenter_s {
	uint seqNr;
	uint fragNr;
	MacDataFrame curr_frame;
	ringbuf frame_queue;
	uint bytes_sent;
	uint bytes_buffered;
} ;

struct MacReassembler_s {
	uint frame_open;
	uint seqNr;
	uint fragNr;
	uint8_t* fragments[MAX_FRAGNR];
	uint fragments_len[MAX_FRAGNR];
	uint frame_len;
} ;


MacFrag mac_frag_init()
{
	MacFrag frag = calloc(1,sizeof(struct MacFragmenter_s));
	frag->frame_queue = ringbuf_create(MAC_MSG_BUF_SIZE);
	frag->curr_frame = NULL;
	return frag;
}

int mac_frag_add_frame(MacFrag frag, MacDataFrame frame)
{
	if (ringbuf_isfull(frag->frame_queue)) {
		LOG(WARN,"[MAC FRAG] cannot enqueue frame. queue full\n");
		return 0;
	} else {
		ringbuf_put(frag->frame_queue,frame);
		frag->bytes_buffered += frame->size;
		return 1;
	}
}

int mac_frag_has_fragment(MacFrag frag)
{
	if (ringbuf_isempty(frag->frame_queue) && (frag->curr_frame==NULL)) {
		return 0;
	} else {
		return 1;
	}
}

int mac_frag_get_buffersize(MacFrag frag)
{
	if (frag->curr_frame) {
		return (frag->curr_frame->size - frag->bytes_sent) + frag->bytes_buffered;
	} else {
		return frag->bytes_buffered;
	}
}

MacMessage mac_frag_get_fragment(MacFrag frag, uint max_frag_size, uint is_uplink)
{
	uint bytes_remain = 0, final_flag,data_len;
	MacMessage fragment = NULL;

	if (frag->curr_frame) {
		// there is a open frame that is being fragmented
		bytes_remain  = frag->curr_frame->size - frag->bytes_sent;
	} else {
		// fetch new frame from queue
		MacDataFrame sdu = ringbuf_get(frag->frame_queue);
		if (sdu == NULL) {
			LOG(ERR,"[MAC FRAG] cannot fetch any SDU from buf\n");
			return NULL;
		}
		frag->bytes_buffered -= sdu->size;
		frag->curr_frame = sdu;
		frag->fragNr = 0;
		frag->seqNr = (frag->seqNr + 1) % MAX_SEQNR;
		frag->bytes_sent = 0;
		bytes_remain = sdu->size;
	}

	// get fragment size and final flag
	if (max_frag_size > bytes_remain + mac_msg_get_hdrlen(ul_data)) {
		data_len = bytes_remain;
		final_flag = 1;
	} else {
		data_len = max_frag_size - mac_msg_get_hdrlen(ul_data);
		final_flag = 0;
	}

	// create MAC Message
	if (is_uplink) {
		fragment = mac_msg_create_ul_data(data_len,final_flag,frag->seqNr,
										  frag->fragNr++,frag->curr_frame->data);
	} else {
		fragment = mac_msg_create_dl_data(data_len,final_flag,frag->seqNr,
										  frag->fragNr++,frag->curr_frame->data);
	}

	// update fragmenter state
	frag->bytes_sent += data_len;
	if (final_flag) {
		free(frag->curr_frame->data);
		free(frag->curr_frame);
		frag->curr_frame = NULL;
	}

	return fragment;
}

MacAssmbl mac_assmbl_init()
{
	MacAssmbl assmbl = calloc(sizeof(struct MacReassembler_s),1);
	return assmbl;
}

MacDataFrame mac_assmbl_reassemble(MacAssmbl assmbl, MacMessage fragment)
{
	MacDLdata* data = &fragment->hdr.DLdata;
	MacDataFrame frame;

	if (!assmbl->frame_open) {
		// no frame open yet, store the received sequence number
		if (data->fragNr == 0) {
			assmbl->seqNr = data->seqNr;
			assmbl->fragNr = 0;
			assmbl->frame_open = 1;
		} else {
			LOG(DEBUG,"[MAC ASSMBL] unexpected fragNr for new frame. "
									"Got %d Expect 0\n",data->fragNr);
			return NULL;
		}
	}

	// ensure that the sequence number and fragment number matches
	// TODO implement unordered fragment reception
	if ((assmbl->seqNr == data->seqNr) && (assmbl->fragNr == data->fragNr)) {
		assmbl->fragments[assmbl->fragNr] = malloc(fragment->payload_len);
		memcpy(assmbl->fragments[assmbl->fragNr],fragment->data,fragment->payload_len);
		assmbl->fragments_len[assmbl->fragNr] = fragment->payload_len;
		assmbl->frame_len += fragment->payload_len;
		assmbl->fragNr++;
	} else {
		// reset reassembler state
		LOG(DEBUG,"[MAC ASSMBL] seq/frag Nr does not match: Got seqNr %d fragNr %d, "
				  "expect %d:%d\n",data->seqNr,data->fragNr,assmbl->seqNr,assmbl->fragNr);
		for (int i=0; i<assmbl->fragNr; i++) {
			free(assmbl->fragments);
		}
		assmbl->frame_open = 0;
		assmbl->frame_len = 0;
		assmbl->fragNr = 0;
	}

	if (data->final_flag) {
		frame = dataframe_create(assmbl->frame_len);
		uint8_t* p = frame->data;
		for (int i=0; i<assmbl->fragNr; i++) {
			memcpy(p, assmbl->fragments[i],assmbl->fragments_len[i]);
			p+=assmbl->fragments_len[i];
			free(assmbl->fragments[i]);
		}
		assmbl->fragNr = 0;
		assmbl->frame_open = 0;
		assmbl->frame_len = 0;
		return frame;
	} else {
		// no complete frame received yet
		return NULL;
	}
}
