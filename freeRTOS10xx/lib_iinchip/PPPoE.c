#include <stdint.h>
#include "PPPoE.h"


#ifdef __DEF_WIZCHIP_PPP__

// ID and Password for PAP from main.c
uint8_t pppoe_id[];
uint8_t pppoe_id_len;
uint8_t pppoe_ip[];
uint8_t pppoe_pw[];
uint8_t pppoe_pw_len;


// PPPoE Frame structure for send
PPPMSG     PPPMSG_req; // PPPoE frame
PROTOCOL   PPPPROTO; // Tag and Protocol data

// TxRx Buffers pointer from main
uint8_t* buf;

// Server MAC and Assigned Session ID and IP address from NAS using PPPoE
uint8_t    NAS_mac[6];
uint16_t   NAS_sessionid = 0;

// kind of authentication protocol and algorithm; decided by LCP phase
// Authentication protocol : PAP - 0xC023, CHAP - 0xC223
// Algorithm : MD5 - 0x05, MS-CHAP - 0x80, MS-CHAP-V2 - 0x81
uint16_t  auth_protocol;
uint8_t   chap_algorithm;

// For MD5 calculation
md5_ctx   context;
uint8_t   digest[16];

// Identifier for PPPoE Protocols (increase per message sending)
uint8_t   protocol_id = 0x01;



uint16_t   pppoe_state = PPPoE_DISCOVERY;
// PPPoE stage control flags
uint16_t   pppoe_control_flag = 0;

//PPPoE retry count and send retry count
uint8_t   pppoe_retry_send_count = 0;
uint16_t  pppoe_retry_count;
uint8_t  pppoe_recv_count = 0;


// Tmp variable
uint16_t   tmp_protocol;
uint8_t   tmp_pcode;

void set_pppinfo(uint8_t * nas_mac, uint8_t * ppp_ip, uint16_t nas_sessionid)
{

#ifdef __DEF_PPP_DBG1__
	uint8_t str[8];//debug var
	uint16_t psid;
	printf("set_pppinfo() Start...\r\n");
#endif
	/* Set PPPoE bit in MR(Common Mode Register) : enable PPPoE */
	setMR(getMR() | MR_PPPOE);

	// Write PPPoE server's MAC address, Session ID and IP address.
	// must be setted these value.
#ifdef __DEF_PPP_DBG1__
	printf("Server's MAC : %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\r\n", nas_mac[0], nas_mac[1], nas_mac[2], nas_mac[3], nas_mac[4], nas_mac[5]);
	printf("PPPoE IP : %.3d.%.3d.%.3d.%.3d\r\n", ppp_ip[0], ppp_ip[1], ppp_ip[2], ppp_ip[3]);
	printf("Session ID : 0x%.2x%.2x\r\n", (uint8_t)(nas_sessionid >> 8), (uint8_t)nas_sessionid);
#endif

	setPHAR(nas_mac);
	setSIPR(ppp_ip);
	setPSID(nas_sessionid);
#ifdef __DEF_PPP_DBG1__
	getPHAR(str);
	printf( "Read PHAR register : %.2x:%.2x:%.2x:%.2x:%.2x:%.2x\r\n", str[0], str[1], str[2], str[3], str[4], str[5]);
	getSIPR(str);
	printf( "Read SIP register : %.3d.%.3d.%.3d.%.3d\r\n", str[0], str[1], str[2], str[3]);
	psid = getPSID();
	printf("Read PSID register : %x\r\n", psid);
#endif


	//open socket in pppoe mode
	//why not close macraw socket and open??
	WIZCHIP_WRITE(PTIMER,200);    // 5sec timeout
	//IINCHIP_WRITE_SOCKETREG(0, Sn_MR,Sn_MR_PPPOE);
	//IINCHIP_WRITE_SOCKETREG(0, Sn_CR,Sn_CR_OPEN);
	//while( WIZCHIP_READ(Sn_CR) );
	//Delay_us(1);

#ifdef __DEF_PPP_DBG1__
	printf("set_pppinfo() End...\r\n");
#endif

}


void ppp_send(void)
{
	uint8_t  *ptr;
	uint8_t  sn = 0;
	uint16_t tmp16 = 0;
	uint16_t txbuf_len = 0;

	int i = 0;

	txbuf_len = sizeof(PPPMSG_req);



	if(pppoe_state == PPPoE_DISCOVERY)
		ptr = (uint8_t *)&PPPPROTO.opt;
	else
		ptr = (uint8_t *)&PPPPROTO;


	// Fill the Tx buffer
	//memcpy(txbuf, (uint8_t *)&PPPMSG_req, txbuf_len);
	memcpy(buf, (uint8_t *)&PPPMSG_req, txbuf_len);
	//change MSB and LSB because of different endian.
	tmp16 = 0;
	tmp16 = (PPPMSG_req.len & 0xFF) << 8;
	tmp16 |= ((PPPMSG_req.len >> 8) & 0xFF);

	//memcpy(txbuf + txbuf_len, ptr, tmp16);
	memcpy(buf + txbuf_len, ptr, tmp16);
	txbuf_len += tmp16;

#ifdef __DEF_PPP_DBG1__
	printf("Send data : ");
	for(i=0; i<txbuf_len; i++)
	{
		if((i % 16) == 0) printf("\r\n");
		printf("%.2x ", buf[i]);
	}
	printf("\r\n\r\n");
#endif
	// Send MACRAW data
	//wiz_send_data(sn, txbuf, txbuf_len);
	wiz_send_data(sn, buf, txbuf_len);
	setSn_CR(sn, Sn_CR_SEND);
	while( getSn_CR(sn) ) ;


}


void ppp_recv( uint16_t received_len )
{

	uint16_t i;
	uint8_t  sn = 0;
	uint8_t  head[2] ={0,};

	uint16_t  ethertype = 0;
	uint8_t   pppoecode = 0;
	uint16_t  taglen = 0;
	uint16_t  tagname;

	uint8_t   get_protocol_id = 0;
	uint16_t  t_idx = 0, acknak_idx = 0, rjt_idx = 0;
	uint16_t  ppp_tag_len = 0, getlen = 0, opt_len = 0;
	uint8_t   acknak_opt[OPTMSG_LEN];
	uint8_t   rjt_opt[OPTMSG_LEN];
	uint8_t   opt_code;
	uint8_t   ppp_code;

	uint8_t   str[OPTMSG_LEN];
	uint16_t  str_len = 0;
	uint8_t   tmp8 = 0;
	uint8_t   mac[6];

	//reset servicename flag
	pppoe_control_flag = pppoe_control_flag & ~FLAG_PADO_SERVICENAME;


	getSHAR(mac);




	// MACRAW Receive
	//receive header(packet length) of macraw packet

	//printf("parameter received len : %d\r\n", received_len);


	wiz_recv_data(sn, head, 2);

	setSn_CR(sn,Sn_CR_RECV);

	while( getSn_CR(sn) ) ;

	received_len = 0;
	received_len = head[0] << 8;
	received_len = received_len | head[1];
	received_len = received_len - 2;

	wiz_recv_data(sn, buf, received_len);

	setSn_CR(sn,Sn_CR_RECV);

	while( getSn_CR(sn) ) ;


	/*
	received_len = received_len - 2;
	//receive rest data
	wiz_recv_data(sn, buf, received_len);
	setSn_CR(sn,Sn_CR_RECV);
	while( getSn_CR(sn) ) ;
	*/
	/*
	printf("buf : ");
	for(i=0; i<received_len; i++)
	{
		printf("%x ", buf[i]);
	}
	printf("\r\n");
	*/





	// Check the MAC in received packet
	tmp8 = buf[0] - mac[0] + buf[1] - mac[1]	+ buf[2] - mac[2]	+ buf[3] - mac[3]	+ buf[4] - mac[4]	+ buf[5] - mac[5];
	if(tmp8==0)
	{
#ifdef __DEF_PPP_DBG1__
		printf("Received packet  :");
		for(i = 0; i < received_len; i++)
			{
				if((i % 16) == 0) printf("\r\n");
				printf("%.2x ", buf[i]);
			}
		printf("\r\n");
#endif
		ethertype = buf[12];
		ethertype = (ethertype << 8) + buf[13];

		pppoecode = buf[15];

		taglen = buf[18];
		taglen = (taglen << 8) + buf[19];
		ppp_code = buf[22];

		//Check the Ether-Type and Code in received packet
		t_idx = 20;
		switch (ethertype)
		{
		case  PPPoE_DISCOVERY :
			if (pppoecode == PPPoE_PADO)
			{
#ifdef __DEF_PPP_DBG1__
                          printf("PPPoE Discovery: PADO received\r\n\r\n");
#endif
				for(i = 0; i < 6; i++) NAS_mac[i] = buf[6+i];
				// PPPoE Frame
				while(taglen)
				{
					tagname = buf[t_idx];
					tagname = (tagname << 8) + buf[t_idx+1];

					ppp_tag_len = buf[t_idx+2];
					ppp_tag_len = (ppp_tag_len << 8) + buf[t_idx+3];

					// Check option field overflow
					// (OPTMSG_LEN defined maximum option field length.)
					if((acknak_idx + (ppp_tag_len+4)) > OPTMSG_LEN)
					{
#ifdef __DEF_PPP_DBG__
					 	printf("PPPoE Protocol option field overflow occuerd!\r\n");
#endif
						break;
					}
					else
					{
						switch(tagname)
						{
						case PPPoED_SERVICE_NAME :
							if ((pppoe_control_flag & FLAG_PADO_SERVICENAME) == 0)
							{
								memcpy(&acknak_opt[acknak_idx], &buf[t_idx], ppp_tag_len+4);
								acknak_idx += (ppp_tag_len+4);
								pppoe_control_flag = pppoe_control_flag | FLAG_PADO_SERVICENAME;
							}
							break;
						case PPPoED_HOST_UNIQ :
						case PPPoED_AC_COOKIE :
							memcpy(&acknak_opt[acknak_idx], &buf[t_idx], ppp_tag_len+4);
							acknak_idx += (ppp_tag_len+4);
							break;
						default :
						//case PPPoED_AC_NAME :
							break;
						}
					}
					t_idx += (ppp_tag_len+4);
					taglen -= (ppp_tag_len+4);
				}

			  	memcpy(&PPPPROTO.opt[0], &acknak_opt[0], acknak_idx);

				for(i = 0; i < 6; i++)
				{
					PPPMSG_req.dst_mac[i] = NAS_mac[i];		// NAS MAC address
				}
				PPPMSG_req.frame_code = PPPoE_PADR;
				//change MSB and LSB because of different endian.
				PPPMSG_req.len = 0;
				PPPMSG_req.len = (acknak_idx & 0xFF) << 8;
				PPPMSG_req.len |= ((acknak_idx >> 8) & 0xFF);

#ifdef __DEF_PPP_DBG1__
				printf("PPPoE Discovery : PADR send\r\n");
#endif
				ppp_send();
				pppoe_control_flag = pppoe_control_flag | FLAG_DISCOVERY_RCV_PADO;


			}
			else if(pppoecode == PPPoE_PADS)
			{
#ifdef __DEF_PPP_DBG1__
				printf("PPPoE Discovery: PADS received\r\n\r\n");
#endif
				pppoe_control_flag = pppoe_control_flag | FLAG_DISCOVERY_RCV_PADS;

				NAS_sessionid = buf[16];
				NAS_sessionid = (NAS_sessionid << 8) + buf[17];
			}
#ifdef __DEF_PPP_DBG1__
			else printf("Not necessary packet received\r\n");
#endif
			break;
		case  PPPoE_SESSION :
			// Process LCP
			if ((buf[20] == 0xc0) && (buf[21] == 0x21))
			{
				//change MSB and LSB because of different endian.
				PPPPROTO.protocol = 0;
				PPPPROTO.protocol = (PPPoE_LCP & 0xFF) << 8;
				PPPPROTO.protocol |= ((PPPoE_LCP >> 8) & 0xFF);

				switch (ppp_code)
				{
				// when lcp_cr_rcv flag set && lcp_cr_sent flag set, goto PAP or CHAP
				case PPP_CONFIG_REQ : //Configuration Request receive, and then ack or reject send
				// when ack sent, lcp_cr_rcv flag set
#ifdef __DEF_PPP_DBG1__
					printf("PPPoE Session LCP: Configure-Request received\r\n\r\n");
#endif
					get_protocol_id = buf[23];
					getlen = buf[24];
					getlen = (getlen<<8) + buf[25];

					getlen -= 4;
					t_idx = 26;
					while (getlen)
					{
						opt_code = buf[t_idx];
						opt_len = buf[t_idx+1];
						// Check option field overflow
						// (OPTMSG_LEN defined maximum option field length.)
						if((acknak_idx + opt_len) > OPTMSG_LEN || (rjt_idx + opt_len) > OPTMSG_LEN)
						{
#ifdef __DEF_PPP_DBG__
					 		printf("PPPoE Protocol option field overflow occuerd!\r\n");
#endif
							break;
						}
						else
						{
							  switch (opt_code)
							  {
							  case LCP_MRU : // MRU (Maximum-Receive-Unit)
							  case LCP_AUTH : // Authentication-Protocol
									auth_protocol = buf[t_idx+2];
									auth_protocol = (auth_protocol << 8) + buf[t_idx+3];
									chap_algorithm = buf[t_idx+4];
							  case LCP_MAGICNUM : // Magic-Number
							  	// opt_code : 0x01, 0x03, 0x05 shared process part
									memcpy(&acknak_opt[acknak_idx], &buf[t_idx], opt_len);
									acknak_idx += opt_len;
								break;
							  default :
							  	memcpy(&rjt_opt[rjt_idx], &buf[t_idx], opt_len);
									rjt_idx += opt_len;
							  	break;
							  }
						  }
						  t_idx += opt_len;
						  getlen -= opt_len;
					}



					if (rjt_idx)
					{
						// reject send, then wait cr
						PPPPROTO.pcode = PPP_CONFIG_REJ; // Reject
					  	memcpy(&PPPPROTO.opt[0], &rjt_opt[0], rjt_idx);
						PPPPROTO.id = get_protocol_id;

						//change MSB and LSB because of different endian.
						PPPPROTO.len = 0;
						PPPPROTO.len = ((rjt_idx+4) & 0xFF) << 8;
						PPPPROTO.len |= (((rjt_idx+4) >> 8) & 0xFF);

						//change MSB and LSB because of different endian.
						PPPMSG_req.len = 0;
						PPPMSG_req.len = ((rjt_idx+6) & 0xFF) << 8;
						PPPMSG_req.len |= (((rjt_idx+6) >> 8) & 0xFF);

						ppp_send();
					}
					else
					{
						// ack send, lcp_cr_rcv flag set
						//change MSB and LSB because of different endian.
						PPPPROTO.protocol = 0;
						PPPPROTO.protocol = (PPPoE_LCP & 0xFF) << 8;
						PPPPROTO.protocol |= ((PPPoE_LCP >> 8) & 0xFF);

						PPPPROTO.pcode = PPP_CONFIG_ACK; // ack
						memcpy(&PPPPROTO.opt[0], &acknak_opt[0], acknak_idx);
						PPPPROTO.id = get_protocol_id;
						//change MSB and LSB because of different endian.
						PPPPROTO.len = 0;
						PPPPROTO.len = ((acknak_idx+4) & 0xFF) << 8;
						PPPPROTO.len |= (((acknak_idx+4) >> 8) & 0xFF);

						//change MSB and LSB because of different endian.
						PPPMSG_req.len = 0;
						PPPMSG_req.len = ((acknak_idx+6) & 0xFF) << 8;
						PPPMSG_req.len |= (((acknak_idx+6) >> 8) & 0xFF);
#ifdef __DEF_PPP_DBG1__
						printf("LCP Configuration Ack send\r\n");
#endif
						ppp_send();//ack send
						pppoe_control_flag = pppoe_control_flag | FLAG_LCP_CR_RCV;

					}
					break;


				case PPP_CONFIG_ACK : //ack, then lcp_cr_sent flag set
#ifdef __DEF_PPP_DBG1__
					printf("PPPoE Session LCP: Configuration Ack received\r\n\r\n");
#endif
					pppoe_control_flag = pppoe_control_flag | FLAG_LCP_CR_SNT;
					break;


				case PPP_TERM_ACK :
#ifdef __DEF_PPP_DBG1__
					printf("Termination Ack received\r\n\r\n");
#endif
					pppoe_control_flag = pppoe_control_flag | FLAG_TERMINATION_ACK_RCV;

					break;


				case PPP_TERM_REQ:
#ifdef __DEF_PPP_DBG1__
					printf("Termination request received\r\n\r\n");
#endif

					//change MSB and LSB because of different endian.
					PPPMSG_req.ether_type = 0;
					PPPMSG_req.ether_type = (PPPoE_SESSION & 0xFF) << 8;
					PPPMSG_req.ether_type |= ((PPPoE_SESSION >> 8) & 0xFF);// session

					PPPMSG_req.frame_code = 0x00;			// session data
					//change MSB and LSB because of different endian.
					PPPMSG_req.session_id = 0;
					PPPMSG_req.session_id = (NAS_sessionid & 0xFF) << 8;
					PPPMSG_req.session_id |= ((NAS_sessionid >> 8) & 0xFF);

					//change MSB and LSB because of different endian.
					PPPPROTO.protocol = 0;
					PPPPROTO.protocol = (PPPoE_LCP & 0xFF) << 8;
					PPPPROTO.protocol |= ((PPPoE_LCP >> 8) & 0xFF);

					PPPPROTO.pcode = PPP_TERM_ACK; 	// Terminate-ack

					PPPPROTO.opt[0] = 0x00;	// Magic number
					PPPPROTO.opt[1] = 0x01;
					PPPPROTO.opt[2] = 0x02;
					PPPPROTO.opt[3] = 0x03;

					PPPPROTO.id = protocol_id++;
					//change MSB and LSB because of different endian.
					PPPPROTO.len = 0;
					PPPPROTO.len = (8 & 0xFF) << 8;
					PPPPROTO.len |= ((8 >> 8) & 0xFF);

					//change MSB and LSB because of different endian.
					PPPMSG_req.len = 0;
					PPPMSG_req.len = (10 & 0xFF) << 8;
					PPPMSG_req.len |= ((10 >> 8) & 0xFF);


#ifdef __DEF_PPP_DBG1__
					printf("Termination ack send\r\n");
#endif

					ppp_send();

					printf("<<<<PPPoE disconnected>>>>\r\n");

					break;
/*
* Notice :  This part is not implemented.
*			If necessary, please implement more for reply for request from NAS.
*
				case 0x04 : //reject
					break;

				case 0x09 : // Echo-Response
					// Backup
					tmp_protocol = PPPPROTO.protocol;
					tmp_pcode = PPPPROTO.pcode;

					PPPPROTO.protocol = PPPoE_LCP;
					PPPPROTO.pcode = PPP_ECHO_REP;

					PPPPROTO.id = buf[23];

					PPPPROTO.len = buf[24];
					PPPPROTO.len = (PPPPROTO.len << 8) + buf[25];

					memcpy(&PPPPROTO.opt[0], &buf[26], PPPPROTO.len-4);
					ppp_send();

					// Recover
					PPPPROTO.protocol = tmp_protocol;
					PPPPROTO.pcode = tmp_pcode;
					break;
*/

				default:
#ifdef __DEF_PPP_DBG1__
					printf("Not necessary packet received\r\n");
#endif
					break;
				}
			}
			// Process PAP
			else if ((buf[20] == 0xc0) && (buf[21] == 0x23))
			{
				if(ppp_code == PPP_CONFIG_ACK) // PPP_CONFIG_ACK = 0x02
				{
					pppoe_control_flag = pppoe_control_flag | FLAG_PAP_ACK_RCV;// receice ack

#ifdef __DEF_PPP_DBG1__
				printf("PPPoE Session PAP: Authentication ACK received\r\n\r\n");
#endif
				}

			}
			// Process CHAP

			else if ((buf[20] == 0xc2) && (buf[21] == 0x23))
			{
				switch(chap_algorithm)
				{
				case MD5 : // 0x05, MD5
#ifdef __DEF_PPP_DBG1__
					printf("PPPoE Session CHAP: CHAP Challenge received\r\n\r\n");
#endif
					get_protocol_id = buf[23];
					// length of [code ~ packet end]
					getlen = buf[24];
					getlen = (getlen<<8) + buf[25];
					// so, 'CHAP data' length is getlen - length of [code(1), ID(1), Length(2)].
					getlen -= 4; //PPP CHAP total(getlen) - 4(header) = data length(value size(1) + value + name )
					t_idx = 26;

					switch(ppp_code)
					{
						case 0x01 : // Challenge, parse the packet and return 'Response' to NAS
							//-- MD5 calc input consist of ID(1), password(pwlen) and Challenge value(16).

							//-- buf[t_idx] => CV(challenge value) size, buf[t_idx+1] => start byte of CV
							memcpy(&str[str_len], &buf[t_idx+1], buf[t_idx]); 	// CV(value)
							str_len += buf[t_idx];
							memcpy(&str[str_len], &pppoe_pw, pppoe_pw_len); 		// user password
							str_len += pppoe_pw_len;
							memcpy(&str[str_len], &pppoe_id, pppoe_id_len); 		// user id
							str_len += pppoe_id_len;


							// MD5 calculation
							// Input - str, str_len / Output - digest
							md5_init(&context);
							md5_update(&context, str, str_len);
							md5_final(digest, &context);

							// making response msg
							acknak_opt[acknak_idx++] = CV_HV_LEN; // fixed value, 16
							memcpy(&acknak_opt[acknak_idx], &digest, CV_HV_LEN);
							acknak_idx += CV_HV_LEN;


							memcpy(&acknak_opt[acknak_idx], &pppoe_id, pppoe_id_len); // Name: User ID
							acknak_idx += pppoe_id_len;


							//change MSB and LSB because of different endian.
							PPPPROTO.protocol = 0;
							PPPPROTO.protocol = (PPPoE_CHAP & 0xFF) << 8;
							PPPPROTO.protocol |= ((PPPoE_CHAP >> 8) & 0xFF);

							PPPPROTO.pcode = 0x02; // response
							PPPPROTO.id = get_protocol_id;
							//change MSB and LSB because of different endian.
							PPPPROTO.len = 0;
							PPPPROTO.len = ((acknak_idx + 4) & 0xFF) << 8;
							PPPPROTO.len |= (((acknak_idx + 4) >> 8) & 0xFF);

							//change MSB and LSB because of different endian.
							PPPMSG_req.len = 0;
							PPPMSG_req.len = ((acknak_idx + 6) & 0xFF) << 8;
							PPPMSG_req.len |= (((acknak_idx + 6) >> 8) & 0xFF);


							memcpy(&PPPPROTO.opt[0], &acknak_opt[0], acknak_idx);
#ifdef __DEF_PPP_DBG1__
							printf("CHAP response MSG send \r\n\r\n");
#endif
							ppp_send();
							break;
						case 0x03 : // PPP_SUCCESS
#ifdef __DEF_PPP_DBG1__
							printf("PPPoE Session CHAP: CHAP Succeed packet received\r\n\r\n");
#endif
							pppoe_control_flag = pppoe_control_flag | FLAG_CHAP_SUC_RCV;

							break;
						case 0x04 : // PPP_FAIL
#ifdef __DEF_PPP_DBG1__
							printf("PPPoE Session CHAP: CHAP failure packet received\r\n\r\n");
#endif

							break;
						default :
							break;
					}
					break;
#ifdef __DEF_PPP_DBG1__
				case MS_CHAP : // 0x80, MS-CHAP
				case MS_CHAP_V2 : // 0x81, MS-CHAP-V2
					printf("This CHAP Algorithm is not supported : ");
					if(chap_algorithm == MS_CHAP) printf("MS-CHAP\r\n");
					else printf("MS-CHAP-V2\r\n");
					break;
#endif
				default :
					break;
				}
			}
			// Process IPCP
			else if ((buf[20] == 0x80) && (buf[21] == 0x21))
			{
				switch(ppp_code)
				{
				case PPP_CONFIG_REQ : // cr, send ack
				case PPP_CONFIG_NAK : // nak, save ip addr and send config requset
#ifdef __DEF_PPP_DBG1__
					if(ppp_code == 0x01) printf("PPPoE Session IPCP: Configure-Request received\r\n\r\n");
					else if (ppp_code == 0x03) printf("PPPoE Session IPCP: Configure-Nak received\r\n\r\n");
#endif
					get_protocol_id = buf[23];
					getlen = buf[24];
					getlen = (getlen<<8) + buf[25];

					getlen -= 4;
					t_idx = 26;


					while (getlen)
					{
						opt_code = buf[t_idx];
						opt_len = buf[t_idx+1];

						// Check option field overflow
						// (OPTMSG_LEN defined maximum option field length.)
						if((acknak_idx + opt_len) > OPTMSG_LEN || (rjt_idx + opt_len) > OPTMSG_LEN)
						{
#ifdef __DEF_PPP_DBG__
						 	printf("PPPoE Protocol option field overflow occuerd!\r\n");
#endif
							break;
						}
						else
						{
							switch (opt_code)
							{
							case 0x02 : // type : ip compression protocol
							case 0x03 : // type : ip address
								memcpy(&acknak_opt[acknak_idx], &buf[t_idx], opt_len);
								if(opt_code == 0x03)
								{
									memcpy(&pppoe_ip, &buf[t_idx+2], opt_len-2);
								}
								acknak_idx += opt_len;
								break;
							default : // reject
								memcpy(&rjt_opt[rjt_idx], &buf[t_idx], opt_len);
								rjt_idx += opt_len;
								break;
							}
						}
						t_idx += opt_len;
						getlen -= opt_len;
					}

					if (rjt_idx)//if reject
					{
#ifdef __DEF_PPP_DBG1__
						 	printf("reject send!\r\n");
#endif
						// reject send, then wait cr
						//change MSB and LSB because of different endian.
						PPPPROTO.protocol = 0;
						PPPPROTO.protocol = (PPPoE_IPCP & 0xFF) << 8;
						PPPPROTO.protocol |= ((PPPoE_IPCP >> 8) & 0xFF);

						PPPPROTO.pcode = 0x04; // Reject

					  memcpy(&PPPPROTO.opt[0], &rjt_opt[0], rjt_idx);

						PPPPROTO.id = get_protocol_id;
						//change MSB and LSB because of different endian.
						PPPPROTO.len = 0;
						PPPPROTO.len = ((rjt_idx+4) & 0xFF) << 8;
						PPPPROTO.len |= (((rjt_idx+4) >> 8) & 0xFF);
						//change MSB and LSB because of different endian.
						PPPMSG_req.len = 0;
						PPPMSG_req.len = ((rjt_idx+6) & 0xFF) << 8;
						PPPMSG_req.len |= (((rjt_idx+6) >> 8) & 0xFF);

						ppp_send();
					}
					else // if not reject
					{
						// ack send, lcp_cr_rcv flag set
						//change MSB and LSB because of different endian.
						PPPPROTO.protocol = 0;
						PPPPROTO.protocol = (PPPoE_IPCP & 0xFF) << 8;
						PPPPROTO.protocol |= ((PPPoE_IPCP >> 8) & 0xFF);

						if(ppp_code == 0x01)
						{
							PPPPROTO.pcode = 0x02; // if configuration request MSG received, send ack
#ifdef __DEF_PPP_DBG1__
						 	printf("ACK send!\r\n");
#endif
						}
						else
						{
							PPPPROTO.pcode = 0x01; // if nak received, send cr
#ifdef __DEF_PPP_DBG1__
						 	printf("Configuration Request send!\r\n");
#endif
						}

					  	memcpy(&PPPPROTO.opt[0], &acknak_opt[0], acknak_idx);

						PPPPROTO.id = get_protocol_id;
						//change MSB and LSB because of different endian.
						PPPPROTO.len = 0;
						PPPPROTO.len = ((acknak_idx+4) & 0xFF) << 8;
						PPPPROTO.len |= (((acknak_idx+4) >> 8) & 0xFF);
						//change MSB and LSB because of different endian.
						PPPMSG_req.len = 0;
						PPPMSG_req.len = ((acknak_idx+6) & 0xFF) << 8;
						PPPMSG_req.len |= (((acknak_idx+6) >> 8) & 0xFF);

						ppp_send();
						if(ppp_code == 0x01) pppoe_control_flag = pppoe_control_flag | FLAG_IPCP_CR_RCV;
						else pppoe_control_flag = pppoe_control_flag | FLAG_IPCP_NAK_RCV;
					}
					break;
				case PPP_CONFIG_ACK : // ack, then ipcp_cr_snt flag set
#ifdef __DEF_PPP_DBG1__
					printf("PPPoE Session IPCP: Configure-Ack received\r\n");
#endif
					if((pppoe_control_flag & FLAG_IPCP_NAK_RCV) == FLAG_IPCP_NAK_RCV) pppoe_control_flag = pppoe_control_flag | FLAG_IPCP_CR_SNT;
					break;
				}
			}
			break;
		default:
			break;
		}
	}
}



void do_discovery(void)
{
	uint16_t i = 0;
	uint8_t mac[6] = {0,};
	getSHAR(mac);

	// PPPoE Frame
	for(i = 0; i < 6; i++)
	{
		PPPMSG_req.dst_mac[i] = 0xFF;			// Broadcast MAC address
		PPPMSG_req.src_mac[i] =	mac[i];		// Source MAC address
		//-- Opt. Device MAC Address
		PPPPROTO.opt[10+i] = mac[i];
	}

	//change MSB and LSB because of different endian.
	PPPMSG_req.ether_type = 0;
	PPPMSG_req.ether_type = (PPPoE_DISCOVERY & 0xFF) << 8;//session data
	PPPMSG_req.ether_type |= ((PPPoE_DISCOVERY >> 8) & 0xFF);//session data

	PPPMSG_req.version_type = PPPoE_VER_TYPE; // Ver : 0001, Type : 0001
	PPPMSG_req.frame_code = PPPoE_PADI; // PADI

	PPPMSG_req.session_id = 0;


	// Tag name : PPPoED_SERVICE_NAME
	PPPPROTO.opt[0] = 0x01;
	PPPPROTO.opt[1] = 0x01;
  	// Tag len
  	PPPPROTO.opt[2] = 0x00;
  	PPPPROTO.opt[3] = 0x00;

	// Tag name : PPPoED_HOST_UNIQ
 	PPPPROTO.opt[4] = 0x01;
  	PPPPROTO.opt[5] = 0x03;
	// Tag len (2bytes)
  	PPPPROTO.opt[6] = 0x00;
  	PPPPROTO.opt[7] = 0x08;
  	// Fill the Host-Uniq field using MAC address
  	PPPPROTO.opt[8] = 0x00;
  	PPPPROTO.opt[9] = 0x00;
  	//PPPPROTO.opt[10~15] refer to "//-- Opt. Device MAC Address"

  	//change MSB and LSB because of different endian.
  	PPPMSG_req.len = 0;
  	PPPMSG_req.len = (16 & 0xFF) << 8;
  	PPPMSG_req.len |= ((16 >> 8) & 0xFF);//size of opt[0-9]+ opt[10-15]

#ifdef __DEF_PPP_DBG1__
	printf("PPPoE Discovery : PADI send\r\n");
#endif
	ppp_send();

}

void do_lcp(void)
{

	//change MSB and LSB because of different endian.
	PPPMSG_req.ether_type = 0;
	PPPMSG_req.ether_type = (PPPoE_SESSION & 0xFF) << 8;
	PPPMSG_req.ether_type |= ((PPPoE_SESSION >> 8) & 0xFF);//session data

	PPPMSG_req.frame_code = 0x00; //session data
	//change MSB and LSB because of different endian.
	PPPMSG_req.session_id = 0;
	PPPMSG_req.session_id = (NAS_sessionid & 0xFF) << 8;
	PPPMSG_req.session_id |= ((NAS_sessionid >> 8) & 0xFF);//session data

	//change MSB and LSB because of different endian.
	PPPPROTO.protocol = 0;
	PPPPROTO.protocol = (PPPoE_LCP & 0xFF) << 8;
	PPPPROTO.protocol |= ((PPPoE_LCP >> 8) & 0xFF);


	PPPPROTO.pcode = PPP_CONFIG_REQ; // CR

	PPPPROTO.opt[0] = 0x05;
	PPPPROTO.opt[1] = 0x06;
	PPPPROTO.opt[2] = 0x00;
	PPPPROTO.opt[3] = 0x01;
	PPPPROTO.opt[4] = 0x02;
	PPPPROTO.opt[5] = 0x03;

	PPPPROTO.id = protocol_id++;
	//change MSB and LSB because of different endian.
	PPPPROTO.len = 0;
	PPPPROTO.len = (10 & 0xFF) << 8;
	PPPPROTO.len |= ((10 >> 8) & 0xFF);

	//change MSB and LSB because of different endian.
	PPPMSG_req.len = 0;
	PPPMSG_req.len = (12 & 0xFF) << 8;
	PPPMSG_req.len |= ((12 >> 8) & 0xFF);

#ifdef __DEF_PPP_DBG1__
	printf("LCP configuration Request send\r\n");
#endif
	ppp_send();

}

void do_lcp_echo(void)
{

	//change MSB and LSB because of different endian.
	PPPMSG_req.ether_type = 0;
	PPPMSG_req.ether_type = (PPPoE_SESSION & 0xFF) << 8;
	PPPMSG_req.ether_type |= ((PPPoE_SESSION >> 8) & 0xFF);// session

	PPPMSG_req.frame_code = 0x00;			// session data
	//change MSB and LSB because of different endian.
	PPPMSG_req.session_id = 0;
	PPPMSG_req.session_id = (NAS_sessionid & 0xFF) << 8;
	PPPMSG_req.session_id |= ((NAS_sessionid >> 8) & 0xFF);// session id

	//change MSB and LSB because of different endian.
	PPPPROTO.protocol = 0;
	PPPPROTO.protocol = (PPPoE_LCP & 0xFF) << 8;
	PPPPROTO.protocol |= ((PPPoE_LCP >> 8) & 0xFF);

	PPPPROTO.pcode = PPP_ECHO_REQ; 	// Echo-Requset

	PPPPROTO.opt[0] = 0x00;	// Magic number
	PPPPROTO.opt[1] = 0x01;
	PPPPROTO.opt[2] = 0x02;
	PPPPROTO.opt[3] = 0x03;

	PPPPROTO.id = protocol_id++;
	//change MSB and LSB because of different endian.
	PPPPROTO.len = 0;
	PPPPROTO.len = (8 & 0xFF) << 8;
	PPPPROTO.len |= ((8 >> 8) & 0xFF);
	//change MSB and LSB because of different endian.
	PPPMSG_req.len = 0;
	PPPMSG_req.len = (10 & 0xFF) << 8;
	PPPMSG_req.len |= ((10 >> 8) & 0xFF);
#ifdef __DEF_PPP_DBG1__
	printf("LCP echo request send\r\n");
#endif
	ppp_send();

}

//return --    1 : success, 0 : failed
uint8_t do_lcp_terminate(void)
{
	uint16_t received_len = 0;
	uint8_t sock_num = 0;

	//change MSB and LSB because of different endian.
	PPPMSG_req.ether_type = 0;
	PPPMSG_req.ether_type = (PPPoE_SESSION & 0xFF) << 8;
	PPPMSG_req.ether_type |= ((PPPoE_SESSION >> 8) & 0xFF);// session

	PPPMSG_req.frame_code = 0x00;			// session data
	//change MSB and LSB because of different endian.
	PPPMSG_req.session_id = 0;
	PPPMSG_req.session_id = (NAS_sessionid & 0xFF) << 8;
	PPPMSG_req.session_id |= ((NAS_sessionid >> 8) & 0xFF);

	//change MSB and LSB because of different endian.
	PPPPROTO.protocol = 0;
	PPPPROTO.protocol = (PPPoE_LCP & 0xFF) << 8;
	PPPPROTO.protocol |= ((PPPoE_LCP >> 8) & 0xFF);

	PPPPROTO.pcode = PPP_TERM_REQ; 	// Terminate-Requset

	PPPPROTO.opt[0] = 0x00;	// Magic number
	PPPPROTO.opt[1] = 0x01;
	PPPPROTO.opt[2] = 0x02;
	PPPPROTO.opt[3] = 0x03;

	PPPPROTO.id = protocol_id++;
	//change MSB and LSB because of different endian.
	PPPPROTO.len = 0;
	PPPPROTO.len = (8 & 0xFF) << 8;
	PPPPROTO.len |= ((8 >> 8) & 0xFF);

	//change MSB and LSB because of different endian.
	PPPMSG_req.len = 0;
	PPPMSG_req.len = (10 & 0xFF) << 8;
	PPPMSG_req.len |= ((10 >> 8) & 0xFF);



	while(1)
	{
#ifdef __DEF_PPP_DBG1__
		printf("Termination Request send\r\n");
#endif
		ppp_send();
		pppoe_retry_count++;

		while( (pppoe_control_flag & FLAG_TERMINATION_ACK_RCV) == 0 )
		{
			//if(pppoe_tick_1s > 3)
			//{

				delay_ms(10);
				if(pppoe_retry_count > PPP_MAX_RETRY_COUNT)
				{
					printf("Termination Failed\r\n");
					return 0;//termination fail
				}


			//}

			received_len = getSn_RX_RSR(sock_num);

			if(received_len > 0)
			{
				ppp_recv(received_len);

				if((pppoe_control_flag & FLAG_TERMINATION_ACK_RCV) == FLAG_TERMINATION_ACK_RCV) // Termination success
				{
					pppoe_state = PPPoE_LCP;

					// Flags reset
					pppoe_control_flag = 0;
					// Clear Session ID for new PPPoE Discovery process
					NAS_sessionid = 0;

					NAS_mac[0] = 0;
					NAS_mac[1] = 0;
					NAS_mac[2] = 0;
					NAS_mac[3] = 0;
					NAS_mac[4] = 0;
					NAS_mac[5] = 0;

					pppoe_ip[0] = 0;
					pppoe_ip[1] = 0;
					pppoe_ip[2] = 0;
					pppoe_ip[3] = 0;

					printf("Termination completed\r\n");
					return 1;//termination success
				}
			}
		}
	}
}


void do_pap(void)
{

	uint16_t   i=0, j=0;

	//change MSB and LSB because of different endian.
	PPPPROTO.protocol = 0;
	PPPPROTO.protocol = (PPPoE_PAP & 0xFF) << 8;
	PPPPROTO.protocol |= ((PPPoE_PAP >> 8) & 0xFF);

	PPPPROTO.pcode = PPP_CONFIG_REQ; 		// cr
	PPPPROTO.opt[i++] = pppoe_id_len;

	for(j = 0; j < pppoe_id_len; j++)
	{
	 	PPPPROTO.opt[i++] = pppoe_id[j];
	}

	PPPPROTO.opt[i++] = pppoe_pw_len;
	for(j = 0; j < pppoe_pw_len; j++)
	{
	 	PPPPROTO.opt[i++] = pppoe_pw[j];
	}

	PPPPROTO.id = protocol_id++;
	//change MSB and LSB because of different endian.
	PPPPROTO.len = 0;
	PPPPROTO.len = ((i+4) & 0xFF) << 8;
	PPPPROTO.len |= (((i+4) >> 8) & 0xFF);
	//change MSB and LSB because of different endian.
	PPPMSG_req.len = 0;
	PPPMSG_req.len = (((i+4)+2) & 0xFF) << 8;
	PPPMSG_req.len |= ((((i+4)+2) >> 8) & 0xFF);

#ifdef __DEF_PPP_DBG1__
	printf("PAP authenticate-Request send\r\n");
#endif

	ppp_send();

}

void do_ipcp(void)
{

	//change MSB and LSB because of different endian.
	PPPPROTO.protocol = 0;
	PPPPROTO.protocol = (PPPoE_IPCP & 0xFF) << 8;
	PPPPROTO.protocol |= ((PPPoE_IPCP >> 8) & 0xFF);

	PPPPROTO.pcode = PPP_CONFIG_REQ; 	// configuration-req

	PPPPROTO.opt[0] = 0x03; 	// option code, IP address
	PPPPROTO.opt[1] = 0x06; 	// option len
 	PPPPROTO.opt[2] = 0x00;
 	PPPPROTO.opt[3] = 0x00;
 	PPPPROTO.opt[4] = 0x00;
 	PPPPROTO.opt[5] = 0x00;

	PPPPROTO.id = protocol_id++;
	//change MSB and LSB because of different endian.
	PPPPROTO.len = 0;
	PPPPROTO.len = (10 & 0xFF) << 8;
	PPPPROTO.len |= ((10 >> 8) & 0xFF);
	//change MSB and LSB because of different endian.
	PPPMSG_req.len = 0;
	PPPMSG_req.len = (12 & 0xFF) << 8;
	PPPMSG_req.len |= ((12 >> 8) & 0xFF);
#ifdef __DEF_PPP_DBG1__
	printf("IPCP Configuration request send\r\n");
#endif

	ppp_send();

}



// PPPoE start
//ret = Success : 1, Fail : 0, continue : 2
uint8_t ppp_start(uint8_t * pppoe_buf)
{
	uint8_t sock_num;
	uint8_t mFlag;
	uint16_t dummyPort;
	uint8_t ret = 2;
	uint16_t received_len = 0;

	uint8_t for_test = 0;


	buf = pppoe_buf;
	//-- Init. param
	sock_num = 0;
	dummyPort = 0;
	mFlag = 0x80; //MAC filter enable in MACRAW



switch(getSn_SR(sock_num))
	{
		case SOCK_CLOSED:
			close(sock_num);										// Close the SOCKET
			socket(sock_num, Sn_MR_MACRAW, dummyPort, mFlag);		// Open the SOCKET with MACRAW mode
#ifdef __DEF_PPP_DBG1__
			printf("No.%d socket is opened with MACRAW and flag is 0x%2x\r\n", sock_num, mFlag);
#endif
#ifdef __DEF_PPP_DBG__
			printf("\r\nPHASE 0. PPPoE setup\r\n");
#endif
		break;

		case SOCK_MACRAW:

			if(pppoe_retry_send_count > PPP_MAX_RETRYSEND_COUNT) pppoe_state = PPP_FAILED;

			switch(pppoe_state)
			{
				case PPPoE_DISCOVERY :											// Discovery
					// PPPoE Discoveryecv(
					if((pppoe_control_flag & FLAG_DISCOVERY_RCV_PADO) == 0)
					{
#ifdef __DEF_PPP_DBG__
						printf("\r\nPHASE 1. PPPoE Discovery\r\n");
#endif
						do_discovery(); // Send PADI
#ifdef __DEF_PPP_DBG1__
						printf("Retry send count : %d\r\n", pppoe_retry_send_count);
#endif
						pppoe_retry_send_count++;
					}
					pppoe_recv_count = 0;
					while((pppoe_control_flag & FLAG_DISCOVERY_RCV_PADS) == 0 && pppoe_recv_count < PPP_MAX_RETRYRECV_COUNT)
					{
						delay_ms(20);
						pppoe_recv_count ++;
						// Recv PADO, Send PADR and Recv PADS
						received_len = getSn_RX_RSR(sock_num);

						if(received_len > 0)
						{
							ppp_recv(received_len);

							if((pppoe_control_flag & FLAG_DISCOVERY_RCV_PADS) == FLAG_DISCOVERY_RCV_PADS)// Discovery success
							{
								pppoe_state = PPPoE_LCP;
								pppoe_retry_send_count = 0;//reset
							}
						}
					}

					break;

				case PPPoE_LCP :												// LCP
					if((pppoe_control_flag & FLAG_LCP_CR_RCV) == 0)
					{
#ifdef __DEF_PPP_DBG__
						printf("\r\nPHASE 2. PPPoE LCP\r\n");
#endif
						do_lcp_echo();

#ifdef __DEF_PPP_DBG1__
						printf("Retry send count : %d\r\n", pppoe_retry_send_count);
#endif
						pppoe_retry_send_count++;
					}

					pppoe_recv_count = 0;
					while((pppoe_control_flag & FLAG_LCP_CR_RCV) == 0 && pppoe_recv_count < PPP_MAX_RETRYRECV_COUNT)
					{
						delay_ms(20);
						pppoe_recv_count++;

						received_len = getSn_RX_RSR(sock_num);
						if(received_len > 0)
						{
							ppp_recv(received_len);
							if((pppoe_control_flag & FLAG_LCP_CR_RCV) == FLAG_LCP_CR_RCV) pppoe_retry_send_count = 0;//reset
						}
					}


					if((pppoe_control_flag & FLAG_LCP_CR_RCV) == FLAG_LCP_CR_RCV)
					{
						do_lcp();

#ifdef __DEF_PPP_DBG1__
						printf("Retry send count : %d\r\n", pppoe_retry_send_count);
#endif
						pppoe_retry_send_count++;

						pppoe_recv_count = 0;
						while((pppoe_control_flag & FLAG_LCP_CR_SNT) == 0 && pppoe_recv_count < PPP_MAX_RETRYRECV_COUNT)
						{
							delay_ms(20);
							pppoe_recv_count++;

							received_len = getSn_RX_RSR(sock_num);
							if(received_len > 0)
							{
								ppp_recv(received_len);
								if((pppoe_control_flag & FLAG_LCP_CR_SNT) == FLAG_LCP_CR_SNT)
								{
									// PAP
									if(auth_protocol == PPPoE_PAP)
									{
										pppoe_state = PPPoE_PAP;
										pppoe_retry_send_count = 0;//reset
									}
									// CHAP
									else if(auth_protocol == PPPoE_CHAP)
									{
										pppoe_state = PPPoE_CHAP;
										pppoe_retry_send_count = 0;//reset
									}
									// unknown auth protocol
									else pppoe_state = PPP_FAILED;

								}
							}
						}
					}
					break;

				case PPPoE_PAP :											   	// PAP
#ifdef __DEF_PPP_DBG__
					printf("\r\nPHASE 3. PPPoE PAP\r\n");
#endif


					if((pppoe_control_flag & FLAG_PAP_ACK_RCV) == 0 )
					{
						do_pap();
						pppoe_retry_send_count++;
#ifdef __DEF_PPP_DBG1__
						printf("Retry send count : %d\r\n", pppoe_retry_send_count );
#endif
					}

					pppoe_recv_count = 0;
					while((pppoe_control_flag & FLAG_PAP_ACK_RCV) == 0 && pppoe_recv_count < PPP_MAX_RETRYRECV_COUNT)
					{
						delay_ms(20);
						pppoe_recv_count++;

						received_len = getSn_RX_RSR(sock_num);
						if(received_len > 0)
						{
							ppp_recv(received_len);

							if((pppoe_control_flag & FLAG_PAP_ACK_RCV) == FLAG_PAP_ACK_RCV)// pap auth success
							{
								pppoe_state = PPPoE_IPCP;
								pppoe_retry_send_count = 0;//reset
							}
						}
					}
					break;

				case PPPoE_CHAP :												// CHAP
#ifdef __DEF_PPP_DBG__
					printf("\r\nPHASE 3. PPPoE CHAP\r\n");
#endif
					pppoe_recv_count = 0;
					while((pppoe_control_flag & FLAG_CHAP_SUC_RCV) == 0 && pppoe_recv_count < PPP_MAX_RETRYRECV_COUNT)
					{
						delay_ms(20);
						pppoe_recv_count++;

						received_len = getSn_RX_RSR(sock_num);
						if(received_len > 0)
						{
							ppp_recv(received_len);

							if((pppoe_control_flag & FLAG_CHAP_SUC_RCV) == FLAG_CHAP_SUC_RCV)
							{
								pppoe_state = PPPoE_IPCP;
							}
						}
					}
					break;

				case PPPoE_IPCP :												// IPCP
#ifdef __DEF_PPP_DBG__
					printf("\r\nPHASE 4. PPPoE IPCP\r\n");
#endif

					pppoe_recv_count = 0;
					while((pppoe_control_flag & FLAG_IPCP_CR_RCV) == 0 && pppoe_recv_count < PPP_MAX_RETRYRECV_COUNT)
					{
						delay_ms(20);
						pppoe_recv_count++;

						received_len = getSn_RX_RSR(sock_num);
						if(received_len > 0)
						{
							ppp_recv(received_len);

							if((pppoe_control_flag & FLAG_IPCP_CR_RCV) == FLAG_IPCP_CR_RCV) pppoe_retry_send_count = 0;//reset

						}

					}
					// After received IPCP Configuration-Request and sent ACK,
					// User device have to send IPCP Configuration-Request and receive ACK.
					if((pppoe_control_flag & FLAG_IPCP_CR_RCV) == FLAG_IPCP_CR_RCV)
					{
						do_ipcp();
						pppoe_retry_send_count++;
#ifdef __DEF_PPP_DBG1__
						printf("Retry send\r\n");
						printf("Retry send count : %d\r\n", pppoe_retry_send_count);
#endif
						pppoe_recv_count = 0;
						while((pppoe_control_flag & FLAG_IPCP_CR_SNT) == 0 && pppoe_recv_count < PPP_MAX_RETRYRECV_COUNT)
						{
							delay_ms(20);
							pppoe_recv_count++;

							received_len = getSn_RX_RSR(sock_num);
							if(received_len > 0)
							{
								ppp_recv(received_len);

								if((pppoe_control_flag & FLAG_IPCP_CR_SNT) == FLAG_IPCP_CR_SNT)
								{
									pppoe_retry_send_count = 0;//reset
#ifdef __DEF_PPP_DBG__
									printf("\r\nPHASE 5. PPPoE Socket open\r\n");
#endif
									set_pppinfo(NAS_mac, pppoe_ip, NAS_sessionid);
									ret = PPP_SUCCESS;
								}

							}
						}
					}
					break;

				case PPP_FAILED :
					pppoe_retry_count++;
#ifdef __DEF_PPP_DBG__
					printf("\r\nPPPoE FAILED !!!\r\n");
					printf("Retry count : %d\r\n", pppoe_retry_count);
#endif
					// All flags reset
					pppoe_control_flag = 0;

					// Clear Session ID for new PPPoE Discovery process
					//NAS_sessionid = 0;

					pppoe_retry_send_count = 0;
					pppoe_state = PPPoE_DISCOVERY;

					break;

				default :
					printf("\r\nUndefined state!\r\n");
					pppoe_state = PPP_FAILED;
					break;
			}

			// PPPoE packet send count over : connection terminate
			if(pppoe_retry_send_count >= PPP_MAX_RETRYSEND_COUNT) pppoe_state = PPP_FAILED;
			break;

		default:
			break;
	}
	return ret;
}

void delay_ms(uint32_t time)
{
	uint32_t i;

	for(i=0; i<time; i++)
	{
		xSysCtlDelay((xSysCtlClockGet()/1000)); // wait 1ms
	}
}

#endif // __DEF_WIZCHIP_PPP__
