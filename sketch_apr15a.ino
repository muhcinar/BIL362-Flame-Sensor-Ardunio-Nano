#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// ========== LCD I2C Ayarlar ==========
#define F_SCL 100000UL
#define Prescaler 1
#define TWBR_val ((((F_CPU / F_SCL) / Prescaler) - 16 ) / 2)
#define LCD_ADDR 0x27

void i2c_init(void) {
    TWSR = 0x00;
    TWBR = (uint8_t)TWBR_val;
    TWCR = (1 << TWEN);
}

void i2c_start(void) {
    TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
    TWCR = (1 << TWSTO) | (1 << TWINT) | (1 << TWEN);
    _delay_us(10);
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWEN) | (1 << TWINT);
    while (!(TWCR & (1 << TWINT)));
}

void lcd_send(uint8_t data, uint8_t mode) {
    uint8_t high = data & 0xF0;
    uint8_t low = (data << 4) & 0xF0;

    i2c_start();
    i2c_write(LCD_ADDR << 1);

    i2c_write(high | mode | 0x08 | 0x04);
    i2c_write(high | mode | 0x08);
    i2c_write(low  | mode | 0x08 | 0x04);
    i2c_write(low  | mode | 0x08);

    i2c_stop();
}

void lcd_command(uint8_t cmd) {
    lcd_send(cmd, 0);
    _delay_ms(2);
}

void lcd_data(uint8_t data) {
    lcd_send(data, 1);
    _delay_ms(2);
}

void lcd_init(void) {
    i2c_init();
    _delay_ms(50);
    lcd_command(0x33);
    lcd_command(0x32);
    lcd_command(0x28);
    lcd_command(0x0C);
    lcd_command(0x06);
    lcd_command(0x01);
    _delay_ms(2);
}

void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t pos[] = {0x80, 0xC0};
    lcd_command(pos[row] + col);
}

void lcd_clear(void) {
    lcd_command(0x01);
    _delay_ms(2);
}

void lcd_print(char* str) {
    while (*str) {
        lcd_data(*str++);
    }
}

// DHT11
#define DHT11_PIN PD2
uint8_t temperature = 0;
uint8_t humidity = 0;

void delay_us(unsigned int us) {
    while (us--){
        _delay_us(1);
    }

}

uint8_t read_dht11() {
    uint8_t data[5] = {0};
    uint8_t i, j;

    DDRD |= (1 << DHT11_PIN); // D Pinini çıkış olarak ayarla
    PORTD &= ~(1 << DHT11_PIN); // D pininden 0 yolla 
    _delay_ms(18);
    PORTD |= (1 << DHT11_PIN); // D pininden 1 yolla 
    delay_us(40);
    DDRD &= ~(1 << DHT11_PIN); // D Pinini giriş olarak ayarla

    _delay_us(10);
    if (PIND & (1 << DHT11_PIN)){ // 1 sinyali geldiyse hatalı dön
      return 1;
    }


    while (!(PIND & (1 << DHT11_PIN))); // 1 sinyalinin 0 a dönmesini bekle
    while (PIND & (1 << DHT11_PIN)); // 0 sinyalinin 1 a dönmesini bekle

    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            while (!(PIND & (1 << DHT11_PIN))); // Okumak için 1 gelene kadar bekle
            delay_us(30);
            if (PIND & (1 << DHT11_PIN)) {
                data[i] |= (1 << (7 - j));
                while (PIND & (1 << DHT11_PIN)); // 
            }
        }
    }

    uint8_t checksum = data[0] + data[1] + data[2] + data[3];
    if (data[4] != checksum) return 2;

    humidity = data[0];
    temperature = data[2];
    return 0;
}

// Buzzer
#define BUZZER_PIN PD6
void buzzer_init() {
    DDRD |= (1 << BUZZER_PIN);
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B = (1 << CS01);
    OCR0A = 128;
}
void buzzer_on()  {
    TCCR0A |= (1 << COM0A1);
}

void buzzer_off() {
    TCCR0A &= ~(1 << COM0A1); 
}

// Flame Sensor (Analog)
#define FLAME_ANALOG_CHANNEL 0 // A0
void adc_init() {
    ADMUX = (1 << REFS0) | (1 << ADLAR);
    ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0);
}
uint16_t read_flame_analog() {
    ADCSRA |= (1 << ADSC);
    while (!(ADCSRA & (1 << ADIF)));
    ADCSRA |= (1 << ADIF);
    return ADCH;
}

//  Ana Program
int main(void) {
    char buffer[16];
    uint8_t status;
    uint16_t flame_level;
    uint8_t flame_percentage;

    lcd_init();
    buzzer_init();
    adc_init();

    while (1) {
        lcd_clear();

        status = read_dht11();
        flame_level = read_flame_analog();

        if (status == 0) {
            flame_percentage = 100 - ((flame_level * 100) / 255);

            lcd_set_cursor(0, 0);
            sprintf(buffer, "S:%uC N:%u%%", temperature, humidity);
            lcd_print(buffer);

            lcd_set_cursor(1, 0);
            sprintf(buffer, "ATES RISKI %3u%%", flame_percentage);
            lcd_print(buffer);
        } else {
            lcd_set_cursor(0, 0);
            lcd_print("DHT11 HATASI!");
            buzzer_off();
            _delay_ms(2000);
            continue;
        }

        if (temperature > 40 || flame_percentage >= 75) {
            lcd_set_cursor(1, 0);
            lcd_print("YANGIN RISKI !!!");
            buzzer_on();
        } else {
            buzzer_off();
        }

        _delay_ms(1000);
    }
}
