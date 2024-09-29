#include <Arduino.h>
#include <esp_wifi.h>

#define TAG "cum"

typedef struct __attribute__((__packed__)){
    uint32_t signature;
    uint32_t version;
    uint8_t message_pair;
    uint8_t essid_len;
    uint8_t essid[32];
    uint8_t keyver;
    uint8_t keymic[16];
    uint8_t mac_ap[6];
    uint8_t nonce_ap[32];
    uint8_t mac_sta[6];
    uint8_t nonce_sta[32];
    uint16_t eapol_len;
    uint8_t eapol[256];
} hccapx_t;
static hccapx_t hccapx = { 
    .signature = 0x58504348, 
    .version = 4, 
    .message_pair = 255,
    .keyver = 2
};

typedef struct {
	uint8_t version;
	uint8_t packet_type;
	uint16_t packet_body_length;
} eapol_packet_header_t;
typedef struct {
	eapol_packet_header_t header;
	uint8_t packet_body[];
} eapol_packet_t;
typedef struct {
    uint8_t key_descriptor_version:3;
    uint8_t key_type:1;
    uint8_t :2;
    uint8_t install:1;
    uint8_t key_ack:1;
    uint8_t key_mic:1;
    uint8_t secure:1;
    uint8_t error:1;
    uint8_t request:1;
    uint8_t encrypted_key_data:1;
    uint8_t smk_message:1;
    uint8_t :2;
} key_information_t;
typedef struct __attribute__((__packed__)) {
    uint8_t descriptor_type;
    key_information_t key_information;
    uint16_t key_length;
    uint8_t key_replay_counter[8];
    uint8_t key_nonce[32];
    uint8_t key_iv[16];
    uint8_t key_rsc[8];
    uint8_t reserved[8];
    uint8_t key_mic[16];
    uint16_t key_data_length;
    uint8_t key_data[];
} eapol_key_packet_t;
typedef struct {
    uint8_t protocol_version:2;
    uint8_t type:2;
    uint8_t subtype:4;
    uint8_t to_ds:1;
    uint8_t from_ds:1;
    uint8_t more_fragments:1;
    uint8_t retry:1;
    uint8_t power_management:1;
    uint8_t more_data:1;
    uint8_t protected_frame:1;
    uint8_t htc_order:1;
} frame_control_t;
typedef struct {
    frame_control_t frame_control;
    uint16_t duration;
    uint8_t addr1[6];
    uint8_t addr2[6];
    uint8_t addr3[6];
    uint16_t sequence_control;
} data_frame_mac_header_t;
typedef struct {
    data_frame_mac_header_t mac_header;
    uint8_t body[];
} data_frame_t;
typedef struct {
    uint8_t snap_dsap;
    uint8_t snap_ssap;
    uint8_t control;
    uint8_t encapsulation[3];
} llc_snap_header_t;
typedef enum {
    EAPOL_EAP_PACKET = 0,
	EAPOL_START,
	EAPOL_LOGOFF,
	EAPOL_KEY,
	EAPOL_ENCAPSULATED_ASF_ALERT,
    EAPOL_MKA,
    EAPOL_ANNOUNCEMENT_GENERIC,
    EAPOL_ANNOUNCEMENT_SPECIFIC,
    EAPOL_ANNOUNCEMENT_REQ
} eapol_packet_types_t;

static unsigned message_ap = 0;
static unsigned message_sta = 0;
static unsigned eapol_source = 0;

_BEGIN_STD_C
static bool is_array_zero(uint8_t *array, unsigned size){
    for(unsigned i = 0; i < size; i++){
        if(array[i] != 0){
            return false;
        }
    }
    return true;
}
eapol_packet_t *parse_eapol_packet(data_frame_t *frame) {
    uint8_t *frame_buffer = frame->body;

    if(frame->mac_header.frame_control.protected_frame == 1) {
        Serial.println("Protected frame, skipping...");
        return NULL;
    }

    if(frame->mac_header.frame_control.subtype > 7) {
        Serial.println("QoS data frame");
        // Skipping QoS field (2 bytes)
        frame_buffer += 2;
    }

    // Skipping LLC SNAP header (6 bytes)
    frame_buffer += sizeof(llc_snap_header_t);

    // Check if frame is type of EAPoL
    if(lwip_htons(*(uint16_t *) frame_buffer) == 0x888E) {
        Serial.println("EAPOL packet");
        frame_buffer += 2;
        return (eapol_packet_t *) frame_buffer; 
    }
    return NULL;
}
static void ap_message_m1(eapol_key_packet_t *eapol_key_packet){
    Serial.println("From AP M1");
    message_ap = 1;
    memcpy(hccapx.nonce_ap, eapol_key_packet->key_nonce, 32);
}
static unsigned save_eapol(eapol_packet_t *eapol_packet, eapol_key_packet_t *eapol_key_packet){
    unsigned eapol_len = 0;
    eapol_len = sizeof(eapol_packet_header_t) + ntohs(eapol_packet->header.packet_body_length);
    if(eapol_len > 256){
        Serial.printf("EAPoL is too long (%u/%u)", eapol_len, 256);
        return 1;
    }
    hccapx.eapol_len = eapol_len;
    memcpy(hccapx.eapol, eapol_packet, hccapx.eapol_len);
    memcpy(hccapx.keymic, eapol_key_packet->key_mic, 16);
    // Clear key MIC from EAPoL packet so hashcat can calulate MIC without preprocessing.
    // This is not documented in HCCAPX reference.
    // But it's based on 802.11i-2004 [8.5.2/h] and by analysing behaviour of cap2hccapx tool
    // MIC key on 77 bytes offset inside EAPoL-Key + 4 bytes EAPoL header.
    memset(&hccapx.eapol[81], 0x0, 16);
    return 0;
}
static void ap_message_m3(eapol_packet_t* eapol_packet, eapol_key_packet_t *eapol_key_packet){
    Serial.println("From AP M3");
    message_ap = 3;
    if(message_ap == 0){
        // No AP message was processed yet. ANonce has to be copied into HCCAPX buffer.
        memcpy(hccapx.nonce_ap, eapol_key_packet->key_nonce, 32);
    }
    if(eapol_source == 2){
        // EAPoL packet was already saved from message #2. No need to resave it.
        hccapx.message_pair = 2;
        return;
    }
    if(save_eapol(eapol_packet, eapol_key_packet) != 0){
        return;
    }
    eapol_source = 3;
    if(message_sta == 2){
        hccapx.message_pair = 3;
    }
}
static void sta_message_m4(eapol_packet_t* eapol_packet, eapol_key_packet_t *eapol_key_packet){
    Serial.println("From STA M4");
    if((message_sta == 2) && (eapol_source != 0)){
        // If message 2 was already fully processed, there is no need to process M4 again 
        Serial.println("Already have M2, not worth");
        return;
    }
    if(message_ap == 0){
        // If there was no AP message processed yet, ANonce will be always missing.
        Serial.println("Not enought handshake messages received.");
        return;
    }
    if(eapol_source == 3){
        hccapx.message_pair = 4;
        return;
    }
    if(save_eapol(eapol_packet, eapol_key_packet) != 0){
        return;
    }
    eapol_source = 4;
    if(message_ap == 1){
        hccapx.message_pair = 1;
    }
    if(message_ap == 3){
        hccapx.message_pair = 5;
    }
}
static void sta_message_m2(eapol_packet_t* eapol_packet, eapol_key_packet_t *eapol_key_packet){
    Serial.println("From STA M2");
    message_sta = 2;
    memcpy(hccapx.nonce_sta, eapol_key_packet->key_nonce, 32);
    if(save_eapol(eapol_packet, eapol_key_packet) != 0){
        return;
    }
    eapol_source = 2;
    if(message_ap == 1){
        hccapx.message_pair = 0;
        return;
    }
}
static void sta_message(data_frame_t *frame, eapol_packet_t* eapol_packet, eapol_key_packet_t *eapol_key_packet){
    if(is_array_zero(hccapx.mac_sta, 6)){
        memcpy(hccapx.mac_sta, frame->mac_header.addr2, 6);
    }
    else if(memcmp(frame->mac_header.addr2, hccapx.mac_sta, 6) != 0){
        Serial.println("Different STA");
        return;
    }
    // Determine which message this is by SNonce
    // SNonce is present in M2, empty in M4
    // Ref: 802.11i-2004 [8.5.3]
    if(!is_array_zero(eapol_key_packet->key_nonce, 16)){
        sta_message_m2(eapol_packet, eapol_key_packet);
    } 
    else {
        sta_message_m4(eapol_packet, eapol_key_packet);
    }
}
static void ap_message(data_frame_t *frame, eapol_packet_t* eapol_packet, eapol_key_packet_t *eapol_key_packet){
    if((!is_array_zero(hccapx.mac_sta, 6)) && (memcmp(frame->mac_header.addr1, hccapx.mac_sta, 6) != 0)){
        Serial.println("Different STA");
        return;
    }
    if(message_ap == 0){
        memcpy(hccapx.mac_ap, frame->mac_header.addr2, 6);
    }
    // Determine which message this is by Key MIC
    // Key MIC is always empty in M1 and always present in M3
    // Ref: 802.11i-2004 [8.5.3]
    if(is_array_zero(eapol_key_packet->key_mic, 16)){
        ap_message_m1(eapol_key_packet);
    } 
    else {
        ap_message_m3(eapol_packet, eapol_key_packet);
    }
}
eapol_key_packet_t *parse_eapol_key_packet(eapol_packet_t *eapol_packet){
    if(eapol_packet->header.packet_type != EAPOL_KEY){
        Serial.println("Not an EAPoL-Key packet.");
        return NULL;
    }
    return (eapol_key_packet_t *) eapol_packet->packet_body;
}
void hccapx_serializer_add_frame(data_frame_t *frame){
    eapol_packet_t *eapol_packet = parse_eapol_packet(frame);
    eapol_key_packet_t *eapol_key_packet = parse_eapol_key_packet(eapol_packet);
    // Determine direction of the frame by comparing BSSID (addr3) with source address (addr2)
    if(memcmp(frame->mac_header.addr2, frame->mac_header.addr3, 6) == 0){
        ap_message(frame, eapol_packet, eapol_key_packet);
    } 
    else if(memcmp(frame->mac_header.addr1, frame->mac_header.addr3, 6) == 0){
        sta_message(frame, eapol_packet, eapol_key_packet);
    } 
    else {
        Serial.println("Unknown frame format. BSSID is not source nor destionation.");
    }
}
_END_STD_C