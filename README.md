# Project overview
This project is designed to control an RC toy car via smartphone application through bluetooth connection.Data is transmitted from the smartphone app to the bluetooth module via bluetooth, and from the bluetooth module to the ATMEGA328 MCU through UART protocol on pin pd0 and pd1.After receiving data, the MCU starts processing it to decide which action to take.

# Tools

* MPLAB X IDE.
* Proteus 8 professional.
  
# Components

* Atmega328.
* L293d motor driver.
* HC-05 bluetooth module.
* 12v battery.
* LM7805 voltage regulator.
* 16MHZ oscillator.
* 0.1uf, 0.01uf and 2x 22pf capacitors.
* 10kohm resistor.

# Connection

The circuit diagram is as follows:

![schematic](https://github.com/Eslam-Rizk/Bluetooth-controlled-RC-car-/blob/main/schematic.png)

# Code

* Defining output pins:

```c
#define LEFT_MOTOR_FORWARD PB0
#define LEFT_MOTOR_BACKWARD PB1
#define RIGHT_MOTOR_FORWARD PB2
#define RIGHT_MOTOR_BACKWARD PB3
```
* Defining void function to initialize Pulse-Width-Modulation for speed control:

```c
void init_pwm(void) {
  // set up Timer/Counter1 for PWM generation on PB1 and PB2
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);  // non-inverted PWM on OCR1A and OCR1B
  TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Fast PWM, prescaler = 8
  ICR1 = F_CPU / (PWM_FREQ * 8);  // set the PWM frequency
  DDRB |= (1 << PB1) | (1 << PB2);  // set PB1 and PB2 as output

  // set up Timer/Counter2 for PWM generation on PB0 and PB3
  TCCR2A |= (1 << COM2A1) | (1 << COM2B1) | (1 << WGM21) | (1 << WGM20);  // non-inverted PWM on OCR2A and OCR2B
  TCCR2B |= (1 << CS21);  // Fast PWM, prescaler = 8
  OCR2A = 0;  // set initial duty cycle to 0%
  OCR2B = 0;
  DDRB |= (1 << PB0) | (1 << PB3);  // set PB0 and PB3 as output
}

```
* Defining a function to set the speed of the motors :

```c
void set_pwm_duty_cycle(uint8_t duty1, uint8_t duty2) {
  OCR1A = (duty1 * PWM_MAX) / 100;  // set the duty cycle as a percentage of the maximum for motor 1
  OCR1B = (duty2 * PWM_MAX) / 100;  // set the duty cycle as a percentage of the maximum for motor 2
}
```

* Defining a void function to initialize UART and to receive data:

```c
void init_uart() {
    // Set baud rate to 9600
    UBRR0H = 0;
    UBRR0L = 103;

    // Enable receiver and transmitter
    UCSR0B |= (1 << RXEN0) | (1 << TXEN0);

    // Set frame format: 8 data bits, 1 stop bit, no parity
    UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}
```
* Set pin as output :

```c
// Set pins for motor control as output
    DDRB |= (1 << LEFT_MOTOR_FORWARD) | (1 << LEFT_MOTOR_BACKWARD) | (1 << RIGHT_MOTOR_FORWARD) | (1 << RIGHT_MOTOR_BACKWARD);

```
* Initialize PWM and set speed as 50% of max speed:

```c
// Initialize PWM for motor control
    init_pwm();
// set duty cycle to 50% for both motors
    set_pwm_duty_cycle(50, 50);
    _delay_ms(1000);
```
* Initialize UART:
```c
// Initialize UART for Bluetooth communication
    init_uart();
```
* Read ncoming data:
```c
// Wait for incoming message
        while (!(UCSR0A & (1 << RXC0)));

        // Read incoming message
        char data = UDR0;
```
* Process received data and take action:
```c
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
```
And you can find the full code [here](https://github.com/Eslam-Rizk/Bluetooth-controlled-RC-car-/blob/main/code/atmega328%20code.c)
