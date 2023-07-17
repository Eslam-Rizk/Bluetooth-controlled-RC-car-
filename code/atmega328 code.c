#include <avr/io.h>
#include <util/delay.h>

#define LEFT_MOTOR_FORWARD PB0
#define LEFT_MOTOR_BACKWARD PB1
#define RIGHT_MOTOR_FORWARD PB2
#define RIGHT_MOTOR_BACKWARD PB3

void init_pwm() {
    // Set timer 0 to fast PWM mode with non-inverted output
    TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
    TCCR0B |= (1 << CS01);

    // Set timer 2 to fast PWM mode with non-inverted output
    TCCR2A |= (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B |= (1 << CS21);
}

void init_uart() {
    // Set baud rate to 9600
    UBRR0H = 0;
    UBRR0L = 103;

    // Enable receiver and transmitter
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

void uart_send_byte(uint8_t data) {
    // Wait for empty transmit buffer
    while (!(UCSR0A & (1 << UDRE0)));

    // Put data into buffer, sends the data
    UDR0 = data;
}

int main() {
    // Set pins for motor control as output
    DDRB |= (1 << LEFT_MOTOR_FORWARD) | (1 << LEFT_MOTOR_BACKWARD) | (1 << RIGHT_MOTOR_FORWARD) | (1 << RIGHT_MOTOR_BACKWARD);

    // Initialize PWM for motor control
    init_pwm();

    // Initialize UART for Bluetooth communication
    init_uart();

    while (1) {
        // Wait for incoming message
        while (!(UCSR0A & (1 << RXC0)));

        // Read incoming message
        char data = UDR0;

        // Parse incoming message
        switch (data) {
            case 'F': // Forward
                PORTB |= (1 << LEFT_MOTOR_FORWARD) | (1 << RIGHT_MOTOR_FORWARD);
                PORTB &= ~((1 << LEFT_MOTOR_BACKWARD) | (1 << RIGHT_MOTOR_BACKWARD));
                break;
            case 'B': // Backward
                PORTB |= (1 << LEFT_MOTOR_BACKWARD) | (1 << RIGHT_MOTOR_BACKWARD);
                PORTB &= ~((1 << LEFT_MOTOR_FORWARD) | (1 << RIGHT_MOTOR_FORWARD));
                break;
            case 'L': // Left
                PORTB |= (1 << LEFT_MOTOR_BACKWARD) | (1 << RIGHT_MOTOR_FORWARD);
                PORTB &= ~((1 << LEFT_MOTOR_FORWARD) | (1 << RIGHT_MOTOR_BACKWARD));
                break;
            case 'R': // Right
                PORTB |= (1 << LEFT_MOTOR_FORWARD) | (1 << RIGHT_MOTOR_BACKWARD);
                PORTB &= ~((1 << LEFT_MOTOR_BACKWARD) | (1 << RIGHT_MOTOR_FORWARD));
                break;
            case 'S': // Stop
                PORTB &= ~((1 << LEFT_MOTOR_FORWARD) | (1 << LEFT_MOTOR_BACKWARD) | (1 << RIGHT_MOTOR_FORWARD) | (1 << RIGHT_MOTOR_BACKWARD));
                break;
            default:
                break;
        }
    }

    return 0;
}