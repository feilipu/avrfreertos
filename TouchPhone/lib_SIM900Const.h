/*
 * lib_SIM900Const.h
 *
 *  Created on: 04/10/2013
 *      Author: phillip
 */

#ifndef LIB_SIM900CONST_H_
#define LIB_SIM900CONST_H_

#include <inttypes.h>

#define GPRS_debug_mode 2

#define SIM900_BUFFER_CMD		64		// command response buffer
#define SIM900_BUFFER_PACKET	1500	// packet receive or transmit (if not otherwise defined), 1460 bytes maximum MTR
#define	SIM900_RATE				115200	// baud rate
#define SIM900_MAX_TIME_OUT		2000	// milli seconds timeout

#define	AT_GPRS_APN		"telstra.internet"
#define	AT_GPRS_LOGIN	"" // "user_name"
#define	AT_GPRS_PASSW	"" // "password"
#define	AT_GPRS_DNS1 	"80.58.0.33"
#define	AT_GPRS_DNS2	"8.8.8.8"

/*********************************************/

#define	AT_COMMAND			"AT"
#define AT_COMMAND_REPEAT	"A//"

#define AT_COMMAND_MODE		"+++"
#define AT_DATA_MODE		"O"
#define AT_DATA_MODE_R		"CONNECT"
#define AT_DATA_MODE_FAIL	"NO CARRIER"
#define OK_RESPONSE 		"OK"
#define ECHO_RESPONSE		"AT"
#define ECHO_OFF_RESPONSE	"ATE0"

//Error Constants
#define ERROR_CME			"+CME ERROR:"
#define ERROR_CMS			"+CMS ERROR:"
#define ERROR				"ERROR"

// Calls Constants
#define AT_CALL			"D" // Start phone call, needs phone number
#define AT_HANG			"H" // Hang-up phone call, no parameters
#define AT_ECHO_OFF		"E0"
#define AT_ANSWER		"A"
#define AT_REDIAL		"DL"
#define AT_DTMF			"+CLDTMF="
#define AT_TONE			"T"
#define AT_PULSE		"P"

// Power Modes Constants
#define AT_POWER_FULL		"+CFUN=1"
#define	AT_POWER_RF_OFF		"+CFUN=4"
#define	AT_POWER_MIN		"+CFUN=0"
#define AT_POWER_OFF_URGNT	"+CPOWD=0"
#define AT_POWER_OFF		"+CPOWD=1"
#define AT_POWER_NO_SLEEP	"+CSCLK=0"
#define AT_POWER_SLEEP		"+CSCLK=2"

#define AT_PRODUCT_ID		"+GSV"
#define AT_REGISTERED		"+CREG?"
#define SET_TIME			"+CCLK="

// SIM Constants
#define AT_PIN				"+CPIN="
#define AT_CHANGE_PASSWORD	"+CPWD="

// Local Data Constants
#define AT_FRAMING			"+ICF"
#define AT_FLOW_CTL			"+IFC"
#define AT_BAUD_RATE		"+IPR"

// IMEI - IMSI Constants
#define AT_GPRS_IMEI 		"+GSN"
#define AT_GPRS_IMEI_R 		"+GSN"
#define AT_GPRS_IMSI 		"+CIMI"

// GPRS Constants
#define AT_GPRS_CGATT		"+CGATT"
#define AT_GPRS_PDP_ACT		"+CGACT"
#define AT_GPRS_DATA		"+CGDATA"
#define	AT_GPRS_CFG			"+SAPBR="
#define	AT_GPRS				"GPRS"
#define	AT_GPRS_IP 			"0.0.0.0"

#define	AT_GPRS_CHECK		"+CGATT?"
#define	AT_GPRS_CHECK_ON	"+CGATT: 1"
#define	AT_GPRS_CHECK_OFF	"+CGATT: 0"
#define	AT_GPRS_ATT_ON		"+CGATT=1"
#define	AT_GPRS_ATT_OFF		"+CGATT=0"
#define AT_NET_REG_STATUS	"+CGREG?"
#define	AT_GPRS_CELLID		"+CENG"
#define AT_GPRS_RSSI		"+CSQ"

//Various
#define AT_GET_OPERATOR			"+COPS?"
#define AT_GET_OPERATOR_R		"+COPS:"
#define AT_SET_PREF_OPERATOR	"+COPS="
#define AT_OPERATOR_LIST		"+CPOL?"
#define AT_OPERATOR_LIST_R		"+CPOL:"
#define AT_WHO_AM_I				"+CGMM"
#define AT_FIRMWARE				"+CGMR"
#define AT_FIRMWARE_R			"Revision:"
#define AT_GET_SP_NAME			"+CSPN?"
#define AT_NUMERIC_ERROR		"+CMEE=1"

#define AT_IP_SET_DNS			"+CDNSCFG="


// Voice Mode Constants
#define	AT_ID_INCALL	"+CLIP"
#define AT_ID_OUTCALL	"+COLP"

// SMS Constants
#define AT_SMS			"+CMGS=" // Set phone number to send SMS to, needs phone number
#define AT_SMS_R		">"
#define AT_SMS_MODE		"+CMGF=1" // Select text mode for SMS
#define	AT_SMS_INFO		"+CNMI=2,1,0,0,0"
#define	AT_SMS_READ		"+CMGR=" // Needs index of SMS to read
#define AT_SMS_MEMORY	"+CPMS="
#define	AT_SMS_MEMORY_R	"+CPMS: "
#define	AT_SMS_DELETE	"+CMGD=" // Needs index of SMS to delete

// Sound Constants
#define AT_SOUND_INT	"#CAP=2" // Set internal audio path
#define AT_SOUND_EXT	"#CAP=1" // Set external audio path
#define AT_VOLUME_SET	"+CLVL=" // Set volume for selected audio path, needs number (min)0..10(MAX)

#define AT_SPEAKER_VOLUME		"L"
#define AT_SPEAKER_MODE			"M"
#define AT_CLIP_MODE			"+CLIP="
#define AT_CLIR_MODE			"+CLIR="
#define AT_PHONE_ACTIVITY		"+CPAS"
#define AT_PHONE_ACTIVITY_R		"+CPAS:"
#define AT_ALERT_SOUND_MODE		"+CALM="
#define AT_ALERT_SOUND_LEVEL	"+CALS="
#define AT_RINGER_SOUND_LEVEL	"+CRSL="
#define AT_SPEAKER_LEVEL		"+CLVL="
#define AT_MUTE					"+CMUT="


// TCP/UDP  constants
#define	AT_IP					"IP"
#define AT_IP_STATUS			"+CIPSTATUS"
#define AT_IP_STATUS_R			"STATE: "
#define AT_IP_SET_START			"+CSTT"
#define AT_IP_SET_APN			"+CSTT="
#define AT_IP_BRING 			"+CIICR"
#define AT_IP_GET_IP			"+CIFSR"
#define AT_IP_CONN_MODE			"+CIPMUX="
#define AT_IP_APP_MODE 			"+CIPMODE="
#define	AT_IP_CLIENT 			"+CIPSTART="
#define AT_TCP					"TCP"
#define AT_UDP					"UDP"
#define AT_CONNECTED_OK			"CONNECT OK"
#define AT_CONNECTED_FAIL		"CONNECT FAIL"
#define	AT_IP_SERVER			"+CIPSERVER="
#define AT_IP_SEND				"+CIPSEND"
#define AT_IP_SEND_R			"SEND OK"
#define AT_IP_SEND_FAIL			"SEND FAIL"
#define	AT_IP_CLOSE				"+CIPCLOSE="
#define AT_IP_CLOSE_R			"CLOSE OK"
#define	AT_IP_QCLOSE			"+CIPQRCLOSE="
#define	AT_IP_SHUT				"+CIPSHUT"
#define	AT_IP_SHUT_R			"SHUT OK"
#define AT_IP_QUERY_DNS			"+CDNSGIP="
#define AT_IP_QUERY_DNS_R_0		"+CDNSGIP: 0"
#define AT_IP_QUERY_DNS_R_1		"+CDNSGIP: 1"
#define AT_IP_LOCAL_PORT		"+CLPORT="
#define AT_IP_SAVE_CONF			"+CIPSCONT"
#define AT_IP_HEADER			"+CIPHEAD="
#define AT_IP_AUTOSENDING		"+CIPATS="
#define AT_IP_SHOW_REMOTE_IP	"+CIPSRIP="
#define AT_IP_PROTOCOL_HEADER	"+CIPSHOWTP="
#define AT_IP_DISCARD_AT_DATA	"+CIPTXISS="
#define AT_IP_GET_MANUALLY		"+CIPRXGET"
#define AT_IP_UDP_EXTENDED		"+CIPUDPMODE="

typedef enum
{
	phone_failure_CME = 0,
	no_connection_to_phone_CME = 1,
	phone_adaptor_link_reserved_CME = 2,
	operation_not_allowed_CME = 3,
	operation_not_supported_CME = 4,
	PH_SIM_PIN_required_CME = 5,
	PH_FSIM_PIN_required_CME = 6,
	PH_FSIM_PUK_required_CME = 7,
	SIM_not_inserted_CME = 10,
	SIM_PIN_required_CME = 11,
	SIM_PUK_required_CME = 12,
	SIM_failure_CME = 13,
	SIM_busy_CME = 14,
	SIM_wrong_CME = 15,
	incorrect_password_CME = 16,
	SIM_PIN2_required_CME = 17,
	SIM_PUK2_required_CME = 18,
	memory_full_CME = 20,
	invalid_index_CME = 21,
	not_found_CME = 22,
	memory_failure_CME = 23,
	text_string_too_long_CME = 24,
	invalid_characters_in_text_string_CME = 25,
	dial_string_too_long_CME = 26,
	invalid_characters_in_dial_string_CME = 27,
	no_network_service_CME = 30,
	network_timeout_CME = 31,
	network_not_allowed_emergency_call_only_CME = 32,
	network_personalisation_PIN_required_CME = 40,
	network_personalisation_PUK_required_CME = 41,
	network_subset_personalisation_PIN_required_CME = 42,
	network_subset_personalisation_PUK_required_CME = 43,
	service_provider_personalisation_PIN_required_CME = 44,
	service_provider_personalisation_PUK_required_CME = 45,
	corporate_personalisation_PIN_required_CME = 46,
	corporate_personalisation_PUK_required_CME = 47,
	resource_limitation_CME = 99,
	unknown_CME = 100,
	Illegal_MS_CME = 103,
	Illegal_ME_CME = 106,
	GPRS_services_not_allowed_CME = 107,
	PLMN_not_allowed_CME = 111,
	Location_area_not_allowed_CME = 112,
	Roaming_not_allowed_in_this_location_area_CME = 113,
	service_option_not_supported_CME = 132,
	requested_service_option_not_subscribed_CME = 133,
	service_option_temporarily_out_of_order_CME = 134,
	unspecified_GPRS_error_CME = 148,
	PDP_authentication_failure_CME = 149,
	invalid_mobile_class_CME = 150,
	Operation_barred_Fixed_dialing_numbers_only_CME = 151,

	ME_failure_CMS = 300,
	SMS_reserved_CMS = 301,
	operation_not_allowed_CMS = 302,
	operation_not_supported_CMS = 303,
	invalid_PDU_mode_CMS = 304,
	invalid_text_mode_CMS = 305,
	SIM_not_inserted_CMS = 310,
	SIM_pin_necessary_CMS = 311,
	PH_SIM_pin_necessary_CMS = 312,
	SIM_failure_CMS = 313,
	SIM_busy_CMS = 314,
	SIM_wrong_CMS = 315,
	SIM_PUK_required_CMS = 316,
	SIM_PIN2_required_CMS = 317,
	SIM_PUK2_required_CMS = 318,
	memory_failure_CMS = 320,
	invalid_memory_index_CMS = 321,
	memory_full_CMS = 322,
	invalid_input_parameter_CMS = 323,
	invalid_input_format_CMS = 324,
	SMSC_address_unknown_CMS = 330,
	no_network_CMS = 331,
	network_timeout_CMS = 332,
	no_cnma_ack_CMS = 340,
	Unknown_CMS = 500,
	SIM_not_ready_CMS = 512,
	unread_records_on_SIM_CMS = 513,
	CB_error_unknown_CMS = 514,
	PS_busy_CMS = 515,
	SIM_BL_not_ready_CMS = 517,
	Invalid_non_hex_chars_in_PDU_CMS = 528,
	Incorrect_PDU_length_CMS = 529,
	Invalid_MTI_CMS = 530,
	Invalid_non_hex_chars_in_address_CMS = 531,
	Invalid_address_no_digits_read_CMS = 532,
	Incorrect_PDU_length_UDL_CMS = 533,
	Incorrect_SCA_length_CMS = 534,
	Invalid_First_Octet_CMS = 536,
	Invalid_Command_Type_CMS = 537,
	SRR_bit_not_set_CMS = 538,
	SRR_bit_set_CMS = 539,
	Invalid_User_Data_Header_IE_CMS = 540,
	missing_required_cmd_parameter_CMS = 753,
	invalid_SIM_command_CMS = 754,
	invalid_File_ID_CMS = 755,
	missing_required_P1_2_3_parameter_CMS = 756,
	invalid_P1_2_3_parameter_CMS = 757,
	missing_required_command_data_CMS = 758,
	invalid_characters_in_command_data_CMS = 759,
	Invalid_input_value_CMS = 765,
	Unsupported_mode_CMS = 766,
	Operation_failed_CMS = 767,
	Mux_already_running_CMS = 768,
	Unable_to_get_control_CMS = 769,
	SIM_network_reject_CMS = 770,
	Call_setup_in_progress_CMS = 771,
	SIM_powered_down_CMS = 772,
	SIM_file_not_present_CMS = 773,

	No_Error = 256
}CME_CMS_ERROR;

#endif /* LIB_SIM900CONST_H_ */
