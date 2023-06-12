/*
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Dirk Ziegelmeier <dziegel@gmx.de>
 *
 */

#include "main.h"
#include "mqtt_mgr.h"

#include "lwip/tcp.h"
#include "lwip/apps/mqtt.h"
#include "mbedtls/debug.h"
#include "lwip/altcp_tls.h"
#include <string.h>

#if LWIP_TCP

/** Define this to a compile-time IP address initialization
 * to connect anything else than IPv4 loopback
 */
#ifndef LWIP_MQTT_EXAMPLE_IPADDR_INIT
#if LWIP_IPV4
// #define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(IPADDR_LOOPBACK))
//  192.168.1.5 || C0 A8 01 05
// #define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(((u32_t)0x0501A8C0UL)))
// broker.hivemq.com || 52.28.53.89
// #define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(((u32_t)0x59351C34UL)))
// #define LWIP_MQTT_EXAMPLE_IPADDR_INIT = {PP_HTONL(LWIP_MAKEU32(52,28,53,89))}
// #define LWIP_MQTT_EXAMPLE_IPADDR_INIT = IPADDR4_INIT(PP_HTONL(((u32_t)0xC0A80105UL)))
#else
#define LWIP_MQTT_EXAMPLE_IPADDR_INIT
#endif
#endif
static ip_addr_t mqtt_ip;
static mqtt_client_t *mqtt_client;
char rcvd_msg_topic[96];

struct tcp_pcb *my_mqttpcb;

const u8_t ca_crt[] =
    {
        "-----BEGIN CERTIFICATE-----\n"
        "MIIDkTCCAnmgAwIBAgIUH75MxFtCG5HJlv2VRwiH8uP8XIIwDQYJKoZIhvcNAQEL\n"
        "BQAwWDELMAkGA1UEBhMCQVUxEzARBgNVBAgMClNvbWUtU3RhdGUxITAfBgNVBAoM\n"
        "GEludGVybmV0IFdpZGdpdHMgUHR5IEx0ZDERMA8GA1UEAwwIUENDTGllbnQwHhcN\n"
        "MjMwNTI4MTAzNTA5WhcNMjgwNjAzMTAzNTA5WjBYMQswCQYDVQQGEwJBVTETMBEG\n"
        "A1UECAwKU29tZS1TdGF0ZTEhMB8GA1UECgwYSW50ZXJuZXQgV2lkZ2l0cyBQdHkg\n"
        "THRkMREwDwYDVQQDDAhQQ0NMaWVudDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCC\n"
        "AQoCggEBAM5RrG3FDM6QrU2ru/mqC6sAUXluZwGa8FrdnEas06ORSyiZ5n++yFhv\n"
        "EBzEgT1oQvMiqhDfxHH3SlZ4xcIvzQvFXeg7mZZ9HbNy2oJNHBgym6nK/IEpNQfv\n"
        "a7/+Z0QTKas22/9+AeEP6x7Rf966Rd30dbi6jGEBeWwOCEj/MgRtGbm+EDI16clY\n"
        "21gBR0KhY0AkNAOeR/id2fRIHKsn29frDseqsV1b4lE1LLaZGZe1MRm7Esn2MOkk\n"
        "sERo7FjuZF8PSQ2lFJNa8ID8VyHt0VZE4qr3arKh0mSQ4eQuTztlcYmmWThKguO7\n"
        "yKjJU9cRBUdpf9SLgDJnh5WSxMzz+g8CAwEAAaNTMFEwHQYDVR0OBBYEFCFaTb7F\n"
        "6cuivOSKI7+ZJhKrjFpoMB8GA1UdIwQYMBaAFCFaTb7F6cuivOSKI7+ZJhKrjFpo\n"
        "MA8GA1UdEwEB/wQFMAMBAf8wDQYJKoZIhvcNAQELBQADggEBADAFblTJQTHqLVqY\n"
        "UciiYOftDxtu7+joQbwNEP5e2rzgRTiEpY1BCttRqQeOPKjJboGiuyCuZLRPtMj/\n"
        "plrlg46yIvKp1XzAZUhwHC2fQI+AZVypB+F1zeA4PETlZApGzPyHCWhdsa9ci71D\n"
        "hZq7RkJFL3HWvRGxmuBMQdXmkqoXM+nh7/fLF326D6xW4v8bGXpg2I08YsSOAvAh\n"
        "OD2+titDTxzcyM5QrfqKjoZHgrADnl9LhhT2TrSAhhcSMJyvLcUeQTqEh3ujJKDG\n"
        "+LK+4mGqn8wxo8yO9ZaX05SDPxgyWVpmRzX25sEB3nig/HpdQtz3PpVGKlLroA6N\n"
        "MU9KiEo=\n"
        "-----END CERTIFICATE-----\n"};

const u8_t amazon_ca_crt[] =
    {
        "-----BEGIN CERTIFICATE-----\n"
        "MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\n"
        "ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\n"
        "b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\n"
        "MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\n"
        "b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\n"
        "ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\n"
        "9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\n"
        "IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\n"
        "VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\n"
        "93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\n"
        "jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\n"
        "AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\n"
        "A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\n"
        "U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\n"
        "N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\n"
        "o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\n"
        "5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\n"
        "rqXRfboQnoZsG4q5WTP468SQvvG5\n"
        "-----END CERTIFICATE-----\n"};

static struct mqtt_connect_client_info_t mqtt_client_info =
    {
        "mqttx_111",
        "test", /* user */
        "test", /* pass */
        100,  /* keep alive */
        NULL, /* will_topic */
        NULL, /* will_msg */
        0,    /* will_qos */
        0     /* will_retain */
#if LWIP_ALTCP && LWIP_ALTCP_TLS
        ,
        NULL
#endif
};

static void
mqtt_incoming_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
  char buff[256];
  LWIP_UNUSED_ARG(arg);
  LWIP_UNUSED_ARG(data);

  sprintf(buff, "%s, %s", (char *)arg, (char *)data);

  mqtt_mgr_send("topic_qos0/echo", buff, strlen(buff));
}

static void
mqtt_incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
  strcpy(arg, topic);
}

static void
mqtt_request_cb(void *arg, err_t err)
{
  const struct mqtt_connect_client_info_t *client_info = (const struct mqtt_connect_client_info_t *)arg;

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" request cb: err %d\r\n", client_info->client_id, (int)err));
}

static void
mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
  const struct mqtt_connect_client_info_t *client_info = (const struct mqtt_connect_client_info_t *)arg;
  LWIP_UNUSED_ARG(client);

  LWIP_PLATFORM_DIAG(("MQTT client \"%s\" connection cb: status %d\r\n", client_info->client_id, (int)status));

  if (status == MQTT_CONNECT_ACCEPTED)
  {

    mqtt_sub_unsub(client,
                   "topic_qos1", 0,
                   mqtt_request_cb, LWIP_CONST_CAST(void *, client_info),
                   1);
    //
    //    mqtt_sub_unsub(client,
    //            "topic_qos0/rcv", 0,
    //            mqtt_request_cb, LWIP_CONST_CAST(void*, client_info),
    //            1);
  }
}
#endif /* LWIP_TCP */

void mqtt_mgr_send(const char *p_topic, const char *p_msg, const uint16_t len)
{

  if ((p_msg != NULL) && (p_msg != NULL) && mqtt_client_is_connected(mqtt_client))
  {
    tcp_write(my_mqttpcb, p_msg, len, 1);
    tcp_output(my_mqttpcb);
    mqtt_publish(mqtt_client, p_topic, p_msg, len, 0, 0, NULL, NULL);
  }
}

void mqtt_mgr_init(void)
{
#if LWIP_TCP
  IP4_ADDR(&mqtt_ip, 192, 168, 1, 5);
  if ((my_mqttpcb = tcp_new()) == NULL)
    return;
  my_mqttpcb->flags |= TF_NODELAY;

  mqtt_client = mqtt_client_new();
#if LWIP_ALTCP && LWIP_ALTCP_TLS
  mbedtls_debug_set_threshold(3);
  mqtt_client_info.tls_config = altcp_tls_create_config_client(NULL, 0);
#endif
  mqtt_client_connect(mqtt_client,
                      &mqtt_ip, MQTT_TLS_PORT,
                      mqtt_connection_cb, LWIP_CONST_CAST(void *, &mqtt_client_info),
                      &mqtt_client_info);

  mqtt_set_inpub_callback(mqtt_client,
                          mqtt_incoming_publish_cb,
                          mqtt_incoming_data_cb,
                          LWIP_CONST_CAST(void *, rcvd_msg_topic));
#endif /* LWIP_TCP */
}
