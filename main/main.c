#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "ssd1306.h"
#include <time.h>
#include <string.h>
#include <math.h>

// Configuraciones del I2C y OLED
#define I2C_PORT I2C_NUM_0
#define SDA_PIN 47
#define SCL_PIN 38
#define OLED_ADDR 0x3C
#define OLED_RST  -1  // Si no usas pin de reset

// Configuraciones del GPS (uart)
#define GPS_UART_NUM     UART_NUM_2
#define GPS_RX_PIN       18
#define GPS_TX_PIN       17
#define GPS_BAUD_RATE    9600
#define GPS_BUF_SIZE     1024

static ssd1306_handle_t display;

// estructura para posiciones GPS
typedef struct {
    double latitude;
    double longitude;
} gps_position_t;

// global para la posición GPS
gps_position_t current_position = {0.0, 0.0};
gps_position_t average_position = {0.0, 0.0};

// arreglo para almacenar posiciones
#define MAX_POSITIONS 100
gps_position_t positions[MAX_POSITIONS];
int position_count = 0;

// Funciones para parsear sentencias gpgsv y sacar número de satélites
int get_satellite_count_from_gpgsv(const char *nmea) {
    if (strncmp(nmea, "$GPGSV", 6) != 0) return -1;

    char copy[128];
    strncpy(copy, nmea, sizeof(copy));
    copy[sizeof(copy) - 1] = '\0';

    char *token = strtok(copy, ",");
    int field = 0;
    while (token != NULL) {
        field++;
        if (field == 3) {
            return atoi(token); // ← número de satélites
        }
        token = strtok(NULL, ",");
    }
    return -1;
}

// Función para parsear sentencias gpgga y sacar número de satélites en uso
int get_satellites_in_use_from_gpgga(const char *nmea) {
    if (strncmp(nmea, "$GPGGA", 6) != 0) return -1;

    char copy[128];
    strncpy(copy, nmea, sizeof(copy));
    copy[sizeof(copy) - 1] = '\0';

    char *token = strtok(copy, ",");
    int field = 0;
    while (token != NULL) {
        field++;
        if (field == 8) {
            return atoi(token); // ← número de satélites en uso
        }
        token = strtok(NULL, ",");
    }
    return -1;
}


char *get_timestamp_from_gpgga(const char *nmea) {
    if (strncmp(nmea, "$GPGGA", 6) != 0) return NULL;

    static char timestamp[10]; // suficiente para "hhmmss.ss"
    char copy[128];
    snprintf(copy, sizeof(copy), "%s", nmea);

    char *token;
    char *rest = copy;
    int field = 0;

    while ((token = strtok_r(rest, ",", &rest))) {
        field++;
        if (field == 2) { // campo 1 es el encabezado, campo 2 es el timestamp
            strncpy(timestamp, token, sizeof(timestamp));
            timestamp[sizeof(timestamp) - 1] = '\0';
            return timestamp;
        }
    }
    return NULL;
}

double get_latitude_from_gpgga(const char *nmea) {
    if (strncmp(nmea, "$GPGGA", 6) != 0) return 0.0;

    char copy[128];
    strncpy(copy, nmea, sizeof(copy));
    copy[sizeof(copy) - 1] = '\0';

    char *token = strtok(copy, ",");
    int field = 0;
    double latitude = 0.0;
    char ns_indicator = 'N';

    while (token != NULL) {
        field++;
        if (field == 3) {
            latitude = atof(token); // ← latitud en formato ddmm.mmmm
        } else if (field == 4) {
            ns_indicator = token[0]; // ← N o S
            break;
        }
        token = strtok(NULL, ",");
    }

    // Convertir a grados decimales
    int degrees = (int)(latitude / 100);
    double minutes = latitude - (degrees * 100);
    double decimal_degrees = degrees + (minutes / 60.0);

    if (ns_indicator == 'S') {
        decimal_degrees = -decimal_degrees;
    }

    return decimal_degrees;
}

double get_longitude_from_gpgga(const char *nmea) {
    if (strncmp(nmea, "$GPGGA", 6) != 0) return 0.0;

    char copy[128];
    strncpy(copy, nmea, sizeof(copy));
    copy[sizeof(copy) - 1] = '\0';

    char *token = strtok(copy, ",");
    int field = 0;
    double longitude = 0.0;
    char ew_indicator = 'E';

    while (token != NULL) {
        field++;
        if (field == 5) {
            longitude = atof(token); // ← longitud en formato dddmm.mmmm
        } else if (field == 6) {
            ew_indicator = token[0]; // ← E o W
            break;
        }
        token = strtok(NULL, ",");
    }

    // Convertir a grados decimales
    int degrees = (int)(longitude / 100);
    double minutes = longitude - (degrees * 100);
    double decimal_degrees = degrees + (minutes / 60.0);

    if (ew_indicator == 'W') {
        decimal_degrees = -decimal_degrees;
    }

    return decimal_degrees;
}

// funcion para calcular la distancia entre dos puntos GPS (Haversine)
double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371e3; // Radio de la Tierra en metros
    double phi1 = lat1 * M_PI / 180.0;
    double phi2 = lat2 * M_PI / 180.0;
    double delta_phi = (lat2 - lat1) * M_PI / 180.0;
    double delta_lambda = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(delta_phi / 2.0) * sin(delta_phi / 2.0) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda / 2.0) * sin(delta_lambda / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1 - a));

    return R * c;
}

void display_task(void *arg) {
    int counter = 0;
    char buffer[32];
    time_t now;
    struct tm timeinfo;
    char time_buffer[32];

    while (1) {
        ssd1306_clear(display);
        ssd1306_draw_text(display, 0, 0, "Hola desde ESP32!", true);

        snprintf(buffer, sizeof(buffer), "Contador: %d", counter);
        ssd1306_draw_text(display, 0, 8, buffer, true);

        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(time_buffer, sizeof(time_buffer), "Hora: %H:%M:%S", &timeinfo);
        ssd1306_draw_text(display, 0, 16, time_buffer, true);

        ssd1306_draw_text(display, 0, 24, "Rod test ESP32.", true);

        ssd1306_display(display);

        counter++;
        vTaskDelay(pdMS_TO_TICKS(5000)); // Ajusta frecuencia de actualización
    }
}

// Tarea para leer datos NMEA del GPS y actualizar el display
// Esta versión procesa solo las sentencias GPGGA para obtener latitud, longitud y número de satélites en uso.
void gps_display_task_nmea(void *arg) {
    uint8_t data[GPS_BUF_SIZE];
    char line[GPS_BUF_SIZE];
    char sat_info[64];
    
    int line_pos = 0;
    int sats=0;

    while (1) {
        int len = uart_read_bytes(GPS_UART_NUM, data, sizeof(data), 100 / portTICK_PERIOD_MS);
        for (int i = 0; i < len; i++) 
        {
            if (data[i] == '\n' || line_pos >= GPS_BUF_SIZE - 1) 
            {
                line[line_pos] = '\0';

                // Si es una gga, la procesa
                if (strncmp(line, "$GPGGA", 6) == 0)
                {
                    // manda el paquete gga a la terminal
                    ESP_LOGI("GPS", "GPGGA: %s", line);

                    // Agarra el numero de sats.
                    sats = get_satellites_in_use_from_gpgga(line);

                    // Agarra el timestamp en texto.
                    char *timestamp_string = get_timestamp_from_gpgga(line);

                    // Número de sats al display junto con el timestamp en una línea.
                    ssd1306_clear(display);
                    snprintf(sat_info, sizeof(sat_info), "S: %d T: %s ", sats, timestamp_string != NULL ? timestamp_string : "N/A");
                    ssd1306_draw_text(display, 0, 0, sat_info, true);

                    // latitud y longitud al display en una línea
                    double latitude = get_latitude_from_gpgga(line);
                    double longitude = get_longitude_from_gpgga(line);
                    snprintf(sat_info, sizeof(sat_info), "%.5f,%.5f", latitude, longitude);
                    ssd1306_draw_text(display, 0, 8, sat_info, true);

                    // Guarda la posición actual
                    current_position.latitude = latitude;
                    current_position.longitude = longitude;

                    // promedia la posición si son menos de cien y la posición tiene más de 3 satélites
                    if (position_count < MAX_POSITIONS && sats > 3) {
                        positions[position_count++] = current_position;

                        double lat_sum = 0.0;
                        double lon_sum = 0.0;
                        for (int j = 0; j < position_count; j++) {
                            lat_sum += positions[j].latitude;
                            lon_sum += positions[j].longitude;
                        }
                        average_position.latitude = lat_sum / position_count;
                        average_position.longitude = lon_sum / position_count;
                    }

                    // si tenemos ya el promedio, desplegamos la distancia al promedio
                    if (position_count == MAX_POSITIONS) {
                        
                        double distance = haversine(
                            current_position.latitude,
                            current_position.longitude,
                            average_position.latitude,
                            average_position.longitude
                        );

                        // Muestra la distancia al promedio solo si la posición es válida usando más de 3 satélites
                       if (sats > 3) {
                           snprintf(sat_info, sizeof(sat_info), "DIST: %.2f m", distance);
                           ssd1306_draw_text(display, 0, 24, sat_info, true);
                       }
                    }

                    // pone el número de la posición actual
                    snprintf(sat_info, sizeof(sat_info), "POS AVG #: %d", position_count);
                    ssd1306_draw_text(display, 0, 16, sat_info, true);

                    // Actualiza el display
                    ssd1306_display(display);
                }

                
                

                line_pos = 0;
            } 
            else if (data[i] != '\r') 
            {
                line[line_pos++] = data[i];
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main(void) {
    // Configurar UART para GPS
    uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0));


    // Configurar bus I2C
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus_handle));

    // Configurar OLED
    ssd1306_config_t cfg = {
        .bus = SSD1306_I2C,
        .width = 128,
        .height = 32,
        .iface.i2c = {
            .port = I2C_PORT,
            .addr = OLED_ADDR,
            .rst_gpio = OLED_RST,
        },
    };
    ESP_ERROR_CHECK(ssd1306_new_i2c(&cfg, &display));

    // Crear tarea para el display
    //xTaskCreate(display_task, "display_task", 2048, NULL, 10, NULL);
    xTaskCreate(gps_display_task_nmea, "gps_display_task_nmea", 8192, NULL, 10, NULL);
    xTaskCreate(display_task, "display_task", 8192, NULL, 10, NULL);

}
