/* [Aug 2014] Cedric Adjih: copied everything from 
   appli/iotlab/node_radio_characterization, and modified from there 

   see also:
   http://www.ietf.org/mail-archive/web/roll/current/pdfa_EzmuDIv3.pdf
*/

#include <stdint.h>
#include <string.h>
#include <inttypes.h> 
#include "printf.h"
#include "scanf.h"
#include "platform.h"
#include "queue.h"
#include "unique_id.h"
#include "crc.h"
#include "debug.h"

#define COMMAND_LEN (256)
enum mode_t { MODE_TX, MODE_RX };

#define CONFIG_RADIO "channel"
#define SEND_PKTS "send"

/* Local variables */
// static version of phy_preprare_packet
static phy_packet_t rx_pkt = {.data = rx_pkt.raw_data };
static phy_packet_t tx_pkt = {.data = tx_pkt.raw_data };
static xQueueHandle cmd_queue;
static xQueueHandle radio_tx_queue;

/* Current configuration */
static struct {
    char power_str[8];
    uint8_t tx_power;
    uint8_t channel;

    uint32_t xmit_id;
    uint16_t pkt_size;
    uint16_t num_pkts;
    uint16_t delay;

    enum mode_t radio_mode;
} conf;


/* Function Protypes */
static void char_handler_irq(handler_arg_t arg, uint8_t c);
static void parse_command_task(void *param);

static void radio_listen();

static void packet_received(handler_arg_t arg);

static int send_one_packet(int num);

/*
 * commands
 */
static void configure_radio(handler_arg_t arg)
{
    int ret = (int)arg;
    if (ret) {
        printf(CONFIG_RADIO " NACK %d\n", ret);
        return;
    }
    conf.radio_mode = MODE_RX;

    phy_idle(platform_phy);

    phy_set_channel(platform_phy, conf.channel);
    radio_listen();

    printf(CONFIG_RADIO " ACK\n");
}

/*---------------------------------------------------------------------------*/

#define MAGIC 0xCEADu

#ifdef __NOT_USED_WITH_CHECKSUM
// bad checksum (to complement CRC)
uint16_t my_checksum(uint8_t* data, uint16_t size)
{
  uint16_t result = 0;
  if ((size & 0x1) != 0) {
    result += data[size-1];
    size --;
  }

  size = size/2;
  int i;
  for (i=0; i<size;i++) {
    uint16_t value = 0;
    memcpy(&value, data+2*i, sizeof(uint16_t));
    result += (value);
  }
  return result;
}
#endif

// XXX: not thread safe (should be ok for this prog.)
static uint32_t get_crc32(uint8_t* data, unsigned int size)
{
  crc_reset();

  static uint32_t tmp_data[32];
  memset(tmp_data, 0, sizeof(tmp_data));
  if (size > sizeof(tmp_data)) {
    log_error("too much data");
    HALT();
  }
  memcpy(tmp_data, data, size);
  crc_compute(tmp_data, (size+3)/4);
  return crc_terminate();

#if 0
  uint32_t data_ptr_value = (uint32_t)data;
  if ((data_ptr_value & 0x3) != 0) { // hack
    log_error("data ptr or content is not 32-bit aligned");
    HALT();
  }

  uint32_t* data32 = (uint32_t*)data_ptr_value;
  unsigned int size32 = size/sizeof(uint32_t);
  crc_compute(data32, size32); 

  unsigned int size_added = size32*sizeof(uint32_t);
  if (size_added != size) {
    uint32_t remainder = 0;
    memcpy(&remainder, data + size_added, size - size_added);
    crc_compute(&remainder, 1);
  }

  return crc_terminate();
#endif  
}

/*---------------------------------------------------------------------------*/

//#define MAX_RECV_PACKET 1024
#define MAX_RECV_PACKET 1048

typedef struct {
  uint8_t lqi;
  int8_t rssi;
  uint16_t seq_num;
  uint32_t timestamp;
  uint32_t eop_time;
} recv_info_t;

#define XMIT_ID_NONE 0
#define SEQ_NUM_OFFSET_ERROR 0xff00u

typedef struct {
  uint32_t xmit_id;
  uint16_t packet_size;

  int nb_packet;
  int nb_change;
  int nb_error;
  int nb_magic_error;
  int nb_crc32_error;
  int nb_locked_error;

  uint8_t locked;
  recv_info_t recv[MAX_RECV_PACKET];
} recv_logger_t;

recv_logger_t recv_logger;

static void recv_logger_init(recv_logger_t* self)
{
  self->nb_packet = 0;
  self->nb_change = 0;
  self->nb_error = 0;
  self->nb_magic_error = 0;
  self->nb_crc32_error = 0;
  self->nb_locked_error = 0;
  self->locked = 0;
  self->xmit_id = XMIT_ID_NONE;
}

static void recv_logger_add_error(recv_logger_t* self,
				  uint32_t timestamp, uint32_t eop_time,
				  uint32_t error_type)
{
  if (self->locked) {
    self->nb_locked_error++;
    return;
  }

  if (self->nb_packet >= MAX_RECV_PACKET) {
    self->nb_error++;
    return;
  }
  recv_info_t* recv_info = &self->recv[self->nb_packet];

  recv_info->seq_num = SEQ_NUM_OFFSET_ERROR+error_type;

  recv_info->rssi = 0;
  recv_info->lqi = 0;
  recv_info->timestamp = timestamp;
  recv_info->eop_time = eop_time;
  self->nb_packet++;
}

static void recv_logger_add(recv_logger_t* self,
			    uint8_t* data, int size, int8_t rssi, uint8_t lqi,
			    uint32_t timestamp, uint32_t eop_time)
{
  if (self->locked) {
    self->nb_locked_error ++;
    return;
  }
  
  if (size < 9) {
    self->nb_error++;
    return;
  }

  uint8_t* initial_data = data;

  uint16_t magic;
  memcpy(&magic, data, sizeof(magic));
  if (magic != MAGIC) {
    recv_logger_add_error(self, timestamp, eop_time, 'M');
    self->nb_magic_error ++;
    return;
  }
  data += sizeof(magic);

  uint32_t value_crc32;
  memcpy(&value_crc32, data, sizeof(value_crc32));
  memset(data, 0, sizeof(value_crc32));
  uint32_t actual_crc32 = get_crc32(initial_data, size);
  if (value_crc32 != actual_crc32) {
    recv_logger_add_error(self, timestamp, eop_time, '3');
    self->nb_crc32_error ++;
    return;
  }
  data += sizeof(value_crc32);

#if 0
  int i;
  printf("recv:");
  for (i=0; i<size; i++)
    printf( "%02x", data[i]);
  printf("\n");
#endif

  uint32_t xmit_id;
  uint16_t packet_id;
  memcpy(&xmit_id, data, sizeof(xmit_id));
  data += sizeof(xmit_id);
  memcpy(&packet_id, data, sizeof(packet_id));
  data += sizeof(packet_id);
  uint8_t packet_size = *data;

  //printf("xmit-id=%u id=%u size=%u\n", xmit_id, packet_id, pkt_size);

  if (packet_size != size) {
    self->nb_error ++;
    return;
  }
    
  if (self->xmit_id != XMIT_ID_NONE) {
    if (self->xmit_id != xmit_id) {
      self->nb_change += 1;
      self->nb_packet = 0;
      self->xmit_id = xmit_id;
      self->packet_size = packet_size;
    }
  } else {
    self->xmit_id = xmit_id;
    self->packet_size = packet_size;
  }

  if (self->packet_size != packet_size)
    self->nb_error++;

  if (self->nb_packet >= MAX_RECV_PACKET) {
    self->nb_error ++;
    return;
  }

  recv_info_t* recv_info = &self->recv[self->nb_packet];
  recv_info->seq_num = packet_id;
  recv_info->rssi = rssi;
  recv_info->lqi = lqi;
  recv_info->timestamp = timestamp;
  recv_info->eop_time = eop_time;
  self->nb_packet++;
}



static void recv_logger_show(recv_logger_t* self)
{
  printf("{'nbPacket':%u,'nbError':%u,'nbMagicError':%u,"
	 "'nbCrc32Error':%u,"
	 "'nbLockedError':%u,"
	 "'nbChange':%u,"
	 "'id':%u,'recv':[", 
	 self->nb_packet, self->nb_error, self->nb_magic_error, 
	 self->nb_crc32_error, self->nb_locked_error,
	 self->nb_change, self->xmit_id);
  int i;
  for (i=0; i<self->nb_packet; i++) {
    if (i > 0)
      printf(",");
    recv_info_t* recv_info = &self->recv[i];
    printf("(%u,%d,%u,%u,%u)", recv_info->seq_num,
	   recv_info->rssi, recv_info->lqi,
	   recv_info->timestamp, recv_info->eop_time);
  }
  printf("]}\n");
}

/*---------------------------------------------------------------------------*/

typedef struct {
  uint32_t timestamp;
  uint32_t eop_time;
  int32_t energy;
  uint8_t success;
} send_info_t;

send_info_t send_table[MAX_RECV_PACKET];

static void send_table_show(int nb_packet, int nb_error, uint32_t xmit_id)
{
  printf("{'nbPacket':%u,'nbError':%u,'id':%u,'send':[", 
	 nb_packet, nb_error, xmit_id);
  int i;
  for (i=0; i<nb_packet; i++) {
    if (i > 0)
      printf(",");
    send_info_t* send_info = &send_table[i];
    printf("(%u,%u,%d,%u)", send_info->timestamp, send_info->eop_time, 
	   send_info->energy, send_info->success);
  }
  printf("]}\n");
}


static void send_packets(handler_arg_t arg)
{
    int ret = (int)arg;

    recv_logger_init(&recv_logger);

    if (ret) {
        printf(SEND_PKTS " NACK %d\n", ret);
        return;
    }

    /* Enter TX mode */
    conf.radio_mode = MODE_TX;
    phy_idle(platform_phy);

    phy_set_power(platform_phy, conf.tx_power);

    /* Send num_pkts packets and count errors */
    int failure_count = 0;
    int i;
    for (i = 0; i < conf.num_pkts; i++) {
        if (send_one_packet(i))
            failure_count++;
        soft_timer_delay_ms(conf.delay);
    }

    /* restart RX */
    conf.radio_mode = MODE_RX;
    radio_listen();

    send_table_show(conf.num_pkts, failure_count, conf.xmit_id);
    
    printf(SEND_PKTS " ACK %u:%u\n", failure_count, conf.num_pkts);
}

// packet format:
// [tx id] [num] [len] ...

static void radio_tx_done(phy_status_t status);
static int send_one_packet(int num)
{
    int success = 0;
    int ret;

    if (num > MAX_RECV_PACKET)
      return 0;

    // setup packet
    uint8_t* data = tx_pkt.data;

#if 0
    // XXX:TODO: true 802.15.4 frame format
    static uint16_t frame_seq_num = 0;
    data[0] = 0x41;
    data[1] = 0x88;
    data[2] = frame_seq_num++;
    data[3] = 0xcd;
    data[4] = 0xab;
    data[5] = 0xff;
    data[6] = 0xff;
    data[7] = hip_get_node_id() & 0xff;
    data[8] = hip_get_node_id() >> 8;
    data += 9;
#endif

    memset(data, num, conf.pkt_size); // fill first (overwrite header below)

    uint16_t magic = MAGIC;
    memcpy(data, &magic, sizeof(magic));
    data += sizeof(uint16_t);

    uint32_t value_crc32 = 0;
    uint8_t* data_crc32_field = data;
    memcpy(data, &value_crc32, sizeof(value_crc32));
    data += sizeof(uint32_t);

    uint16_t packet_id = num;
    memcpy(data, &conf.xmit_id, sizeof(conf.xmit_id));
    data += sizeof(conf.xmit_id);

    memcpy(data, &packet_id, sizeof(packet_id));
    data += sizeof(packet_id);
    *data = conf.pkt_size;

    value_crc32 = get_crc32(tx_pkt.data, conf.pkt_size);
    memcpy(data_crc32_field, &value_crc32, sizeof(value_crc32));

#ifdef CHECK_CORRUPTION
    /* check if crc32 catches packet corruption */
    if (num%10 == 0)
      tx_pkt.data[conf.pkt_size-1] ^= 0xfau;
    //printf("crc32: %u\n", value_crc32);
    value_crc32 = get_crc32(tx_pkt.data, conf.pkt_size);
    //printf("->  crc32: %u\n", value_crc32);
#endif

    tx_pkt.length = conf.pkt_size;

    //--------------------------------------------------
    
    // get energy
    phy_idle(platform_phy);
    phy_status_t status = phy_ed(platform_phy, &send_table[num].energy);
    if (status != PHY_SUCCESS)
      send_table[num].energy = -1111;

    // send packet
    phy_idle(platform_phy);
    phy_tx_now(platform_phy, &tx_pkt, radio_tx_done);
    // wait tx done
    ret  = (pdTRUE == xQueueReceive(radio_tx_queue, &success,
                1000 * portTICK_RATE_MS));
    ret &= success;
    send_table[num].success = ret;
    send_table[num].timestamp = tx_pkt.timestamp;
    send_table[num].eop_time = tx_pkt.eop_time;

    return (ret ? 0 : 1);
}
static void radio_tx_done(phy_status_t status)
{
    int success = (PHY_SUCCESS == status);
    xQueueSend(radio_tx_queue, &success, 0);
}


/*
 * Radio RX
 */
static void packet_received_net_handler(phy_status_t status);
static void radio_listen()
{
    if (MODE_RX == conf.radio_mode) {
        /* keep receiving packets */
        phy_idle(platform_phy);
        phy_rx_now(platform_phy, &rx_pkt, packet_received_net_handler);
    }
}
static void packet_received_net_handler(phy_status_t status)
{
    /* Printf should be done from EVENT_QUEUE_APPLI */
    event_post(EVENT_QUEUE_APPLI, packet_received, (handler_arg_t)status);
}
static void packet_received(handler_arg_t arg)
{
    phy_status_t status = (phy_status_t) arg;
    if (PHY_SUCCESS == status) {
      //printf("radio_rx %s %d %u sender power num rssi lqi\n",
      //rx_pkt.data, rx_pkt.rssi, rx_pkt.lqi);
      recv_logger_add(&recv_logger, rx_pkt.data, rx_pkt.length, rx_pkt.rssi, 
		      rx_pkt.lqi, rx_pkt.timestamp, rx_pkt.eop_time);
    } else if (status == PHY_RX_CRC_ERROR) {
      // from driver: timestamp/eop_time should be filled
      recv_logger_add_error(&recv_logger, 
			    rx_pkt.timestamp, rx_pkt.eop_time, 'C');
    } else if (status == PHY_RX_LENGTH_ERROR) {
      // from driver: timestamp/eop_time should be filled
      recv_logger_add_error(&recv_logger, 
			    rx_pkt.timestamp, rx_pkt.eop_time, 'L');
    } else {
      printf("radio_rx_error 0x%x\n", status);
      //recv_info_list_radio_error(status);
    }
    radio_listen();
}

/*---------------------------------------------------------------------------*/

typedef struct {
  char name[14];
  int code;
} dbm_str_code_t;

#define md(x) {"-"#x, PHY_POWER_m##x##dBm}

dbm_str_code_t dbm_str_code_list [] = {
  md(17), md(12), md(9), md(7), md(5), md(4), md(3), md(2), md(1),
  {"0", PHY_POWER_0dBm },
  {"0.7", PHY_POWER_0_7dBm},
  {"1.3", PHY_POWER_1_3dBm},
  {"1.8", PHY_POWER_1_8dBm},
  {"2.3", PHY_POWER_2_3dBm},
  {"2.8", PHY_POWER_2_8dBm},
  {"3",   PHY_POWER_3dBm},
  { "",   0x2014 /*unused*/ }
};

static unsigned int parse_power(char *power_str)
{
  int i;
  //printf("<power_str='%s'>", power_str);
  for (i=0; dbm_str_code_list[i].name[0] != 0; i++) 
    if (strcmp(power_str, dbm_str_code_list[i].name) == 0)
      return dbm_str_code_list[i].code;
  return 0xffu;
}

/*--------------------------------------------------*/
#if 0
#define mp(x) \
    if (strcmp("-" #x, power_str) == 0) \
        return PHY_POWER_m##x##dBm;


#define pp(x) \
    if (strcmp(#x, power_str) == 0) \
        return PHY_POWER_##x##dBm;

static unsigned int parse_power(char *power_str)
{
    /* valid PHY_POWER_ values for rf231 */
  /* mp(30);  */
  /* mp(29);mp(28);mp(27);mp(26);mp(25);mp(24);mp(23);mp(22);mp(21);mp(20); */
  /* mp(19);mp(18);mp(17);mp(16);mp(15);mp(14);mp(13);mp(12);mp(11);mp(10); */
  /* mp(9); mp(8); mp(7); mp(6); mp(5); mp(4); mp(3); mp(2); mp(1); */

  //pp(0); pp(0_7); pp(1); pp(2); pp(3); pp(4); pp(5)

  mp(17); mp(12); mp(9); mp(7); mp(5); mp(4); mp(3); mp(2); mp(1); 

    if (strcmp("0", power_str) == 0)
        return PHY_POWER_0dBm;
    if (strcmp("0.7", power_str) == 0)
        return PHY_POWER_0_7dBm;

    if (strcmp("1.3", power_str) == 0)
        return PHY_POWER_1_3dBm;
    if (strcmp("1.8", power_str) == 0)
        return PHY_POWER_1_8dBm;

    if (strcmp("2.3", power_str) == 0)
        return PHY_POWER_2_3dBm;
    if (strcmp("2.8", power_str) == 0)
        return PHY_POWER_2_8dBm;

    if (strcmp("3", power_str) == 0)
        return PHY_POWER_3dBm;

    return 255;
};
#endif
/*--------------------------------------------------*/

/*---------------------------------------------------------------------------*/

static void parse_command_task(void *param)
{
    static char command_buffer[COMMAND_LEN];
    char *cmd;
    char *args;

    int ret;
    int valid_command;

    while (1) {
        if (pdTRUE != xQueueReceive(cmd_queue, command_buffer, portMAX_DELAY))
            continue;

        cmd = strtok_r(command_buffer, " \n\r", &args);
        if (cmd == NULL)
            continue;

        if (strcmp(CONFIG_RADIO, cmd) == 0) {

            // "channel <channel>"
            ret = sscanf(args, "%u", &conf.channel);

            valid_command = ((1 == ret) &&
                    (conf.channel >= 11) && (conf.channel <= 26));

            event_post(EVENT_QUEUE_APPLI, configure_radio, (handler_arg_t)
                    (valid_command ? 0 : 1));

        } else if (strcmp(SEND_PKTS, cmd) == 0) {

            // "send <xmit_id> <tx_power:XXdBm> <pkt size> 
	    // <num_packets> <delay_ms>"
            ret = sscanf(args, "%"SCNu32 " %s " "%"SCNu16 " %"SCNu16 
			 " %"SCNu16 "\n",
			 &conf.xmit_id, conf.power_str, &conf.pkt_size,
			 &conf.num_pkts, &conf.delay);
            conf.tx_power =  parse_power(conf.power_str);

	    //printf("txpower=%d ret=%d args='%s' fmt='%s'\n", 
	    //conf.tx_power, ret, args, SCNu32);

            valid_command = ((5 == ret) && (conf.tx_power != 255)
			     && (conf.xmit_id != XMIT_ID_NONE));

            event_post(EVENT_QUEUE_APPLI, send_packets, (handler_arg_t)
                    (valid_command ? 0 : 1));

	} else if (strcmp("get_tx_power_list", cmd) == 0) {

	  printf("[");
	  int i;
	  for (i=0; dbm_str_code_list[i].name[0] != 0; i++) {
	    if (i>0) 
	      printf(",");
	    //printf("\"%s\":%d", dbm_str_code_list[i].name,
	    //dbm_str_code_list[i].code);
	    printf("\"%s\"", dbm_str_code_list[i].name);
	  }
	  printf("]\n");

	} else if (strcmp("uid", cmd) == 0) {

	  printf("uid=");
	  int i;
	  for (i=0;i<sizeof(uid->uid8);i++) {
	    if (i>0)
	      printf(":");
	    printf("%02x", uid->uid8[i]);
	  }
	  printf("\n");

	} else if (strcmp("clear", cmd) == 0) {

	  unsigned int nb_packet = recv_logger.nb_packet;
	  recv_logger_init(&recv_logger);
	  printf("clear ACK %u %u\n", nb_packet, soft_timer_time());

	} else if (strcmp("show", cmd) == 0) {

	  recv_logger_show(&recv_logger);

	} else if (strcmp("lock", cmd) == 0) {

	  recv_logger.locked = 1;
	  printf("lock ACK %u\n", soft_timer_time());

        } else {
            printf("invalid_command\n");
            continue;
        }
    }
}


static void char_handler_irq(handler_arg_t arg, uint8_t c)
{
    static char command_buffer[COMMAND_LEN];
    static size_t index = 0;

    portBASE_TYPE yield;

    command_buffer[index++] = c;

    // line full or new line
    if (('\n' == c) || (COMMAND_LEN == index)) {
        command_buffer[index] = '\0';
        index = 0;

        xQueueSendFromISR(cmd_queue, command_buffer, &yield);

        if (yield)
            portYIELD();
    }
}

int main(void) {
    /* Setup the hardware. */
    platform_init();
    soft_timer_init();
    crc_enable();

    cmd_queue      = xQueueCreate(2, 256);  // command queue
    radio_tx_queue = xQueueCreate(8, sizeof(int));

    recv_logger_init(&recv_logger);
    xTaskCreate(parse_command_task, (const signed char *const) "Parse command",
            4 * configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    uart_set_rx_handler(uart_print, char_handler_irq, NULL);

    platform_run();
    return 1;
}
