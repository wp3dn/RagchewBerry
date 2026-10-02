#include "p1_network.h"

#include <string.h>
#include <stdatomic.h>
#include "pico/stdlib.h"
#include "lwip/ip.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "dhcpserver/dhcpserver.h"
#include "usb_network.h"
#include "p1_protocol.h"

static const ip4_addr_t own_ip  = IPADDR4_INIT_BYTES(192, 168, 7, 2);
static const ip4_addr_t netmask = IPADDR4_INIT_BYTES(255, 255, 255, 0);
static const ip4_addr_t gateway = IPADDR4_INIT_BYTES(0, 0, 0, 0);

#define RX_QUEUE_SLOTS 16u
typedef struct {
    int32_t i[P1_SAMPLES_PER_PACKET];
    int32_t q[P1_SAMPLES_PER_PACKET];
} rx_iq_slot_t;

static struct udp_pcb *p1_pcb;
static dhcp_server_t dhcp;
static bool dhcp_started;
static p1_network_status_t status;
static ip_addr_t peer_addr;
static u16_t peer_port;
static bool peer_valid;

static rx_iq_slot_t rx_queue[RX_QUEUE_SLOTS];
static volatile uint8_t rx_write;
static volatile uint8_t rx_read;

#define TX_IQ_RING_FRAMES 4096u
static int16_t tx_iq_ring[TX_IQ_RING_FRAMES * 2u];
static volatile uint16_t tx_iq_write;
static volatile uint16_t tx_iq_read;

static unsigned tx_iq_count(void) {
    return (unsigned)((tx_iq_write - tx_iq_read) & (TX_IQ_RING_FRAMES - 1u));
}

void p1_network_flush_tx_iq(void) {
    __atomic_store_n(&tx_iq_write, 0u, __ATOMIC_RELEASE);
    __atomic_store_n(&tx_iq_read, 0u, __ATOMIC_RELEASE);
}

int p1_network_tx_iq_available(void) { return (int)tx_iq_count(); }
void p1_network_note_tx_underflow(void) { status.tx_iq_underflows++; }

static void queue_tx_iq_frames(const int16_t *iq, int frames) {
    if (!iq || frames <= 0) return;
    uint16_t w = __atomic_load_n(&tx_iq_write, __ATOMIC_RELAXED);
    for (int n = 0; n < frames; ++n) {
        uint16_t r = __atomic_load_n(&tx_iq_read, __ATOMIC_ACQUIRE);
        uint16_t next = (uint16_t)((w + 1u) & (TX_IQ_RING_FRAMES - 1u));
        if (next == r) {
            status.tx_iq_frames_dropped += (uint32_t)(frames - n);
            break;
        }
        tx_iq_ring[w * 2u + 0u] = iq[n * 2 + 0];
        tx_iq_ring[w * 2u + 1u] = iq[n * 2 + 1];
        w = next;
        status.tx_iq_frames_received++;
    }
    __atomic_store_n(&tx_iq_write, w, __ATOMIC_RELEASE);
}

int p1_network_read_tx_iq(int16_t *iq, int max_frames) {
    if (!iq || max_frames <= 0) return 0;
    uint16_t r = __atomic_load_n(&tx_iq_read, __ATOMIC_RELAXED);
    int out = 0;
    while (out < max_frames) {
        uint16_t w = __atomic_load_n(&tx_iq_write, __ATOMIC_ACQUIRE);
        if (r == w) break;
        iq[out * 2 + 0] = tx_iq_ring[r * 2u + 0u];
        iq[out * 2 + 1] = tx_iq_ring[r * 2u + 1u];
        r = (uint16_t)((r + 1u) & (TX_IQ_RING_FRAMES - 1u));
        ++out;
    }
    __atomic_store_n(&tx_iq_read, r, __ATOMIC_RELEASE);
    return out;
}

static void remember_peer(const ip_addr_t *addr, u16_t port) {
    if (!addr) return;
    ip_addr_copy(peer_addr, *addr);
    peer_port = port;
    peer_valid = true;
}

static void send_discovery(struct udp_pcb *pcb, const ip_addr_t *addr, u16_t port) {
    uint8_t reply[60];
    p1_build_discovery_reply(reply, sizeof(reply));
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(reply), PBUF_RAM);
    if (!p) return;
    memcpy(p->payload, reply, sizeof(reply));
    udp_sendto(pcb, p, addr, port);
    pbuf_free(p);
    status.discovered = true;
    status.discovery_count++;
}

static void udp_rx(void *arg, struct udp_pcb *pcb, struct pbuf *p,
                   const ip_addr_t *addr, u16_t port) {
    (void)arg;
    if (!p) return;

    uint8_t header[8] = {0};
    uint16_t n = p->tot_len;
    uint16_t h = n < sizeof(header) ? n : sizeof(header);
    pbuf_copy_partial(p, header, h, 0);
    int kind = p1_classify(header, h);

    if (kind == 1) {
        send_discovery(pcb, addr, port);
    } else if (kind == 3 && n >= P1_PACKET_BYTES) {
        static uint8_t packet[P1_PACKET_BYTES];
        static int16_t iq[P1_SAMPLES_PER_PACKET * 2];
        pbuf_copy_partial(p, packet, sizeof(packet), 0);
        int mox = 0;
        uint32_t tx_hz = 0, rx_hz = 0;
        int frames = p1_unpack_ep2(packet, sizeof(packet), iq,
                                    P1_SAMPLES_PER_PACKET, &mox, &tx_hz, &rx_hz);
        bool old_mox = status.mox_requested;
        status.control_seen = true;
        status.control_count++;
        status.mox_requested = mox != 0;
        if (status.mox_requested && !old_mox) p1_network_flush_tx_iq();
        if (status.mox_requested && frames > 0) queue_tx_iq_frames(iq, frames);
        if (!status.mox_requested && old_mox) p1_network_flush_tx_iq();
        if (rx_hz) status.rx_frequency_hz = rx_hz;
        if (tx_hz) status.tx_frequency_hz = tx_hz;
    } else if (kind == 2) {
        if (n >= 4 && header[0] == 0xEF && header[1] == 0xFE && header[2] == 0x04) {
            /* Johan Hub semantics: 00=STOP, 01/03=START.
               Capture stream destination only on START. */
            bool run = (header[3] == 0x01u || header[3] == 0x03u);
            bool was_streaming = status.rx_streaming;
            status.control_seen = true;
            status.control_count++;
            if (run) remember_peer(addr, port);
            status.rx_streaming = run;
            if (run && !was_streaming) {
                rx_write = rx_read = 0;
                p1_protocol_reset();
            } else if (!run && was_streaming) {
                rx_write = rx_read = 0;
            }
        }
    }
    pbuf_free(p);
}

bool p1_network_queue_rx_iq(const int32_t *i24, const int32_t *q24) {
    if (!status.initialized || !status.rx_streaming || !i24 || !q24) return false;
    uint8_t w = __atomic_load_n(&rx_write, __ATOMIC_RELAXED);
    uint8_t next = (uint8_t)((w + 1u) % RX_QUEUE_SLOTS);
    uint8_t r = __atomic_load_n(&rx_read, __ATOMIC_ACQUIRE);
    if (next == r) {
        status.rx_packets_dropped++;
        return false;
    }
    memcpy(rx_queue[w].i, i24, sizeof(rx_queue[w].i));
    memcpy(rx_queue[w].q, q24, sizeof(rx_queue[w].q));
    atomic_thread_fence(memory_order_release);
    __atomic_store_n(&rx_write, next, __ATOMIC_RELEASE);
    return true;
}

static void send_one_rx_packet(void) {
    if (!p1_pcb || !peer_valid || !status.rx_streaming) return;
    uint8_t r = __atomic_load_n(&rx_read, __ATOMIC_RELAXED);
    uint8_t w = __atomic_load_n(&rx_write, __ATOMIC_ACQUIRE);
    if (r == w) return;

    uint8_t packet[P1_PACKET_BYTES];
    p1_build_ep6(packet, rx_queue[r].i, rx_queue[r].q, NULL);
    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, sizeof(packet), PBUF_RAM);
    if (!p) return;
    memcpy(p->payload, packet, sizeof(packet));
    err_t err = udp_sendto(p1_pcb, p, &peer_addr, peer_port);
    pbuf_free(p);
    if (err == ERR_OK) {
        __atomic_store_n(&rx_read, (uint8_t)((r + 1u) % RX_QUEUE_SLOTS), __ATOMIC_RELEASE);
        status.rx_packets_sent++;
    }
}

bool p1_network_start(void) {
    if (status.initialized) return true;
    memset(&status, 0, sizeof(status));
    p1_protocol_reset();
    rx_write = rx_read = 0;
    p1_network_flush_tx_iq();
    peer_valid = false;

    if (!usb_network_init(&own_ip, &netmask, &gateway, true)) return false;

    dhcp_server_init(&dhcp, (ip_addr_t *)&own_ip, (ip_addr_t *)&netmask, false);
    dhcp_started = true;

    p1_pcb = udp_new();
    if (!p1_pcb) {
        p1_network_stop();
        return false;
    }
    if (udp_bind(p1_pcb, IP_ADDR_ANY, 1024) != ERR_OK) {
        p1_network_stop();
        return false;
    }
    udp_recv(p1_pcb, udp_rx, NULL);
    status.initialized = true;
    return true;
}

void p1_network_stop(void) {
    if (p1_pcb) {
        udp_remove(p1_pcb);
        p1_pcb = NULL;
    }
    if (dhcp_started) {
        dhcp_server_deinit(&dhcp);
        dhcp_started = false;
    }
    if (status.initialized) usb_network_deinit();
    memset(&status, 0, sizeof(status));
    rx_write = rx_read = 0;
    p1_network_flush_tx_iq();
    peer_valid = false;
}

void p1_network_task(void) {
    if (!status.initialized) return;
    usb_network_update();
    status.link_up = usb_network_is_up();
    for (int n = 0; n < 8; ++n) send_one_rx_packet();
}

const p1_network_status_t *p1_network_status(void) { return &status; }
