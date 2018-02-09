#ifndef __DHT11_H
#define __DHT11_H
void delay_us(int32_t count);
void dht11_time_init(void);
uint8_t DHT11_Read_Data(uint8_t *temp,uint8_t *humi);
void DHT11_Prepare(void);
#endif /* __DHT11_H */
