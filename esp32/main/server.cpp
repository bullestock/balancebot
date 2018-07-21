/*
 * Firmware for a segway-style robot using ESP32.
 * Copyright (C) 2017  Sakari Kapanen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <cstring>
#include <utility>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <lwip/api.h>

#include "esp_log.h"

#include "websocket_server.h"

#include "espway.h"
#include "locks.h"
#include "q16.h"

static QueueHandle_t client_queue;
const static int client_queue_size = 10;


static void httpd_websocket_save_config()
{
    uint8_t response = save_flash_config() ? RES_SAVE_CONFIG_SUCCESS : RES_SAVE_CONFIG_FAILURE;
    ws_server_send_bin_all_from_callback(&response, 1);
}

static void httpd_websocket_clear_config()
{
    uint8_t response = RES_CLEAR_CONFIG_FAILURE;
    // Clear the configuration by writing config version zero
    bool success = clear_flash_config();
    if (success)
    {
        response = RES_CLEAR_CONFIG_SUCCESS;
        load_hardcoded_config();
    }
    ws_server_send_bin_all_from_callback(&response, 1);
}

static void send_pid_params(pid_controller_index idx)
{
    uint8_t buf[14];
    buf[0] = RES_PID_PARAMS;
    buf[1] = idx;
    int32_t* params = (int32_t*) (&buf[2]);
    {
        MutexLock lock(pid_mutex);
        params[0] = my_config.pid_coeffs_arr[idx].p;
        params[1] = my_config.pid_coeffs_arr[idx].i;
        params[2] = my_config.pid_coeffs_arr[idx].d;
    }
    ws_server_send_bin_all_from_callback(buf, sizeof(buf));
}

static void send_orientation()
{
    uint8_t buf[5];
    buf[0] = RES_ORIENTATION;
    int16_t* qdata = (int16_t*) &buf[1];
    orientation my_orientation = get_orientation();
    qdata[0] = FLT_TO_Q16(my_orientation.sin_pitch / 2);
    qdata[1] = FLT_TO_Q16(my_orientation.sin_roll / 2);
    ws_server_send_bin_all_from_callback(buf, sizeof(buf));
}

static void httpd_websocket_cb(const uint8_t* data, uint16_t data_len)
{
    if (data_len == 0)
        return;

    uint8_t msgtype = data[0];
    auto payload = &data[1];
    data_len -= 1;
    int8_t* signed_data;
    pid_controller_index pid_index;
    int32_t* i32_data;
    uint8_t res;

    switch (msgtype)
    {
    case STEERING:
        // Parameters: velocity (int8_t), turn rate (int8_t)
        if (data_len != 2) break;
        signed_data = (int8_t*) payload;
        //!!
        set_steering((FLT_TO_Q16(SPEED_CONTROL_FACTOR) * signed_data[1]) / 128,
                     (FLT_TO_Q16(STEERING_FACTOR) * signed_data[0]) / 128);
        break;

    case REQ_ORIENTATION:
        if (data_len != 0)
            break;
        send_orientation();
        break;

    case REQ_SET_PID_PARAMS:
        // Parameters: pid index (uint8_t), P (q16/int32_t), I (q16), D (q16)
        if (data_len != 13) break;

        pid_index = (pid_controller_index)payload[0];
        i32_data = (int32_t*) (&payload[1]);
        if (pid_index < (sizeof(pid_settings_arr) / sizeof(pid_settings_arr[0])))
        {
            update_pid_controller(pid_index, i32_data[0], i32_data[1],
                                  i32_data[2]);
            res = RES_SET_PID_PARAMS_ACK;
            ws_server_send_bin_all_from_callback(&res, 1);
        }

        break;

    case REQ_GET_PID_PARAMS:
        if (data_len != 1)
            break;
        if (payload[0] < (sizeof(pid_settings_arr) / sizeof(pid_settings_arr[0])))
        {
            send_pid_params((pid_controller_index) payload[0]);
        }
        break;

    case REQ_LOAD_FLASH_CONFIG:
        if (data_len != 0)
            break;
        load_config();
        res = RES_LOAD_FLASH_CONFIG_DONE;
        ws_server_send_bin_all_from_callback(&res, 1);
        break;

    case REQ_SAVE_CONFIG:
        if (data_len != 0)
            break;
        httpd_websocket_save_config();
        break;

    case REQ_CLEAR_CONFIG:
        if (data_len != 0)
            break;
        httpd_websocket_clear_config();
        break;
    }
}

// handles websocket events
void websocket_callback(uint8_t num, WEBSOCKET_TYPE_t type, char* msg, uint64_t len)
{
    const static char* TAG = "websocket_callback";
    int value;

    switch(type)
    {
    case WEBSOCKET_CONNECT:
        ESP_LOGI(TAG, "client %i connected!", num);
        break;
    case WEBSOCKET_DISCONNECT_EXTERNAL:
        ESP_LOGI(TAG, "client %i sent a disconnect message", num);
        break;
    case WEBSOCKET_DISCONNECT_INTERNAL:
        ESP_LOGI(TAG, "client %i was disconnected", num);
        break;
    case WEBSOCKET_DISCONNECT_ERROR:
        ESP_LOGI(TAG, "client %i was disconnected due to an error", num);
        break;
    case WEBSOCKET_TEXT:
        if (len)
        {
            switch(msg[0]) {
            case 'L':
                if(sscanf(msg, "L%i", &value)) {
                    ESP_LOGI(TAG, "LED value: %i", value);
                    //led_duty(value);
                    ws_server_send_text_all_from_callback(msg, len); // broadcast it!
                }
            }
        }
        break;
    case WEBSOCKET_BIN:
        httpd_websocket_cb((const uint8_t*) msg, len);
        break;
    case WEBSOCKET_PING:
        ESP_LOGI(TAG, "client %i pinged us with message of size %i:\n%s", num, (uint32_t)len, msg);
        break;
    case WEBSOCKET_PONG:
        ESP_LOGI(TAG, "client %i responded to the ping", num);
        break;
    }
}

// serves any clients
static void http_serve(struct netconn *conn) {
    const static char* TAG = "http_server";
    const static char HTML_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";
    const static char ERROR_HEADER[] = "HTTP/1.1 404 Not Found\nContent-type: text/html\n\n";
    const static char JS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/javascript\n\n";
    const static char ICO_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/x-icon\n\n";
    struct netbuf* inbuf;
    static char* buf;
    static uint16_t buflen;
    static err_t err;

    // index.html
    extern const uint8_t index_html_start[] asm("_binary_index_html_start");
    extern const uint8_t index_html_end[] asm("_binary_index_html_end");
    const uint32_t index_html_len = index_html_end - index_html_start;

    // index.bundle.js
    extern const uint8_t index_bundle_js_start[] asm("_binary_index_bundle_js_start");
    extern const uint8_t index_bundle_js_end[] asm("_binary_index_bundle_js_end");
    const uint32_t index_bundle_js_len = index_bundle_js_end - index_bundle_js_start;

    // pid.html
    extern const uint8_t pid_html_start[] asm("_binary_pid_html_start");
    extern const uint8_t pid_html_end[] asm("_binary_pid_html_end");
    const uint32_t pid_html_len = pid_html_end - pid_html_start;

    // pid.bundle.js
    extern const uint8_t pid_bundle_js_start[] asm("_binary_pid_bundle_js_start");
    extern const uint8_t pid_bundle_js_end[] asm("_binary_pid_bundle_js_end");
    const uint32_t pid_bundle_js_len = pid_bundle_js_end - pid_bundle_js_start;

    // favicon.ico
    extern const uint8_t favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const uint8_t favicon_ico_end[] asm("_binary_favicon_ico_end");
    const uint32_t favicon_ico_len = favicon_ico_end - favicon_ico_start;

    netconn_set_recvtimeout(conn, 1000); // allow a connection timeout of 1 second
    ESP_LOGI(TAG, "reading from client...");
    err = netconn_recv(conn, &inbuf);
    ESP_LOGI(TAG, "read from client");
    if(err==ERR_OK) {
        netbuf_data(inbuf, (void**)&buf, &buflen);
        if(buf) {

            // default page
            if (strstr(buf, "GET / ")
                && !strstr(buf, "Upgrade: websocket")) {
                ESP_LOGI(TAG, "Sending /");
                netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER)-1, NETCONN_NOCOPY);
                netconn_write(conn, index_html_start, index_html_len, NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }

            // default page websocket
            else if((strstr(buf, "GET / ") || strstr(buf, "GET /ws "))
                    && strstr(buf, "Upgrade: websocket")) {
                ESP_LOGI(TAG, "Requesting websocket on /");
                ws_server_add_client(conn, buf, buflen, "/", websocket_callback);
                netbuf_delete(inbuf);
            }

            else if (strstr(buf, "GET /pid") && !strstr(buf, "GET /pid.bundle.js")) {
                ESP_LOGI(TAG, "Sending /pid.html");
                netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER)-1, NETCONN_NOCOPY);
                netconn_write(conn, pid_html_start, pid_html_len, NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
            else if(strstr(buf, "GET /index.bundle.js ")) {
                ESP_LOGI(TAG, "Sending /index.bundle.js");
                netconn_write(conn, JS_HEADER, sizeof(JS_HEADER)-1, NETCONN_NOCOPY);
                netconn_write(conn, index_bundle_js_start, index_bundle_js_len, NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
            else if(strstr(buf, "GET /pid.bundle.js ")) {
                ESP_LOGI(TAG, "Sending /pid.bundle.js");
                netconn_write(conn, JS_HEADER, sizeof(JS_HEADER)-1, NETCONN_NOCOPY);
                netconn_write(conn, pid_bundle_js_start, pid_bundle_js_len, NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }

            else if(strstr(buf, "GET /favicon.ico ")) {
                ESP_LOGI(TAG, "Sending favicon.ico");
                netconn_write(conn, ICO_HEADER, sizeof(ICO_HEADER)-1, NETCONN_NOCOPY);
                netconn_write(conn, favicon_ico_start, favicon_ico_len, NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }

            else if(strstr(buf, "GET /")) {
                ESP_LOGI(TAG, "Unknown GET request: %s", buf);
                netconn_write(conn, ERROR_HEADER, sizeof(ERROR_HEADER)-1, NETCONN_NOCOPY);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }

            else {
                ESP_LOGI(TAG, "Unknown request: %s", buf);
                netconn_close(conn);
                netconn_delete(conn);
                netbuf_delete(inbuf);
            }
        }
        else {
            ESP_LOGI(TAG, "Unknown request (empty?...)");
            netconn_close(conn);
            netconn_delete(conn);
            netbuf_delete(inbuf);
        }
    }
    else { // if err==ERR_OK
        ESP_LOGI(TAG, "error on read, closing connection");
        netconn_close(conn);
        netconn_delete(conn);
        netbuf_delete(inbuf);
    }
}

// handles clients when they first connect. passes to a queue
void server_task(void* pvParameters) {
    const static char* TAG = "server_task";
    struct netconn *conn, *newconn;
    static err_t err;
    client_queue = xQueueCreate(client_queue_size, sizeof(struct netconn*));

    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL,80);
    netconn_listen(conn);
    ESP_LOGI(TAG, "server listening");
    do {
        err = netconn_accept(conn, &newconn);
        ESP_LOGI(TAG, "new client");
        if (err == ERR_OK)
            xQueueSendToBack(client_queue, &newconn, portMAX_DELAY);
    } while(err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
    ESP_LOGE(TAG, "task ending, rebooting board");
    esp_restart();
}

// receives clients from queue, handles them
void server_handle_task(void* pvParameters) {
    const static char* TAG = "server_handle_task";
    struct netconn* conn;
    ESP_LOGI(TAG, "task starting");
    while (1)
    {
        xQueueReceive(client_queue,&conn,portMAX_DELAY);
        if (!conn)
            continue;
        http_serve(conn);
    }
    vTaskDelete(NULL);
}
