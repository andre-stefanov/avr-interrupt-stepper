#pragma once

#ifdef ARDUINO_ARCH_AVR

#include <Arduino.h>

namespace internal
{
    enum Port
    {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        J,
        K,
        L
    };

    template <Port P>
    constexpr inline volatile uint8_t *port_to_output()
    {
        switch (P)
        {
        case A:
            return &PORTA;
        case B:
            return &PORTB;
        case C:
            return &PORTC;
        case D:
            return &PORTD;
        case E:
            return &PORTE;
        case F:
            return &PORTF;
        case G:
            return &PORTG;
        case H:
            return &PORTH;
        case J:
            return &PORTJ;
        case K:
            return &PORTK;
        case L:
            return &PORTL;
        }
    }

    template <uint8_t PIN>
    constexpr inline Port pin_to_port()
    {
        switch (PIN)
        {
        case 0:
            return Port::E; // PE 0 ** 0 ** USART0_RX
        case 1:
            return Port::E; // PE 1 ** 1 ** USART0_TX
        case 2:
            return Port::E; // PE 4 ** 2 ** PWM2
        case 3:
            return Port::E; // PE 5 ** 3 ** PWM3
        case 4:
            return Port::G; // PG 5 ** 4 ** PWM4
        case 5:
            return Port::E; // PE 3 ** 5 ** PWM5
        case 6:
            return Port::H; // PH 3 ** 6 ** PWM6
        case 7:
            return Port::H; // PH 4 ** 7 ** PWM7
        case 8:
            return Port::H; // PH 5 ** 8 ** PWM8
        case 9:
            return Port::H; // PH 6 ** 9 ** PWM9
        case 10:
            return Port::B; // PB 4 ** 10 ** PWM10
        case 11:
            return Port::B; // PB 5 ** 11 ** PWM11
        case 12:
            return Port::B; // PB 6 ** 12 ** PWM12
        case 13:
            return Port::B; // PB 7 ** 13 ** PWM13
        case 14:
            return Port::J; // PJ 1 ** 14 ** USART3_TX
        case 15:
            return Port::J; // PJ 0 ** 15 ** USART3_RX
        case 16:
            return Port::H; // PH 1 ** 16 ** USART2_TX
        case 17:
            return Port::H; // PH 0 ** 17 ** USART2_RX
        case 18:
            return Port::D; // PD 3 ** 18 ** USART1_TX
        case 19:
            return Port::D; // PD 2 ** 19 ** USART1_RX
        case 20:
            return Port::D; // PD 1 ** 20 ** I2C_SDA
        case 21:
            return Port::D; // PD 0 ** 21 ** I2C_SCL
        case 22:
            return Port::A; // PA 0 ** 22 ** D22
        case 23:
            return Port::A; // PA 1 ** 23 ** D23
        case 24:
            return Port::A; // PA 2 ** 24 ** D24
        case 25:
            return Port::A; // PA 3 ** 25 ** D25
        case 26:
            return Port::A; // PA 4 ** 26 ** D26
        case 27:
            return Port::A; // PA 5 ** 27 ** D27
        case 28:
            return Port::A; // PA 6 ** 28 ** D28
        case 29:
            return Port::A; // PA 7 ** 29 ** D29
        case 30:
            return Port::C; // PC 7 ** 30 ** D30
        case 31:
            return Port::C; // PC 6 ** 31 ** D31
        case 32:
            return Port::C; // PC 5 ** 32 ** D32
        case 33:
            return Port::C; // PC 4 ** 33 ** D33
        case 34:
            return Port::C; // PC 3 ** 34 ** D34
        case 35:
            return Port::C; // PC 2 ** 35 ** D35
        case 36:
            return Port::C; // PC 1 ** 36 ** D36
        case 37:
            return Port::C; // PC 0 ** 37 ** D37
        case 38:
            return Port::D; // PD 7 ** 38 ** D38
        case 39:
            return Port::G; // PG 2 ** 39 ** D39
        case 40:
            return Port::G; // PG 1 ** 40 ** D40
        case 41:
            return Port::G; // PG 0 ** 41 ** D41
        case 42:
            return Port::L; // PL 7 ** 42 ** D42
        case 43:
            return Port::L; // PL 6 ** 43 ** D43
        case 44:
            return Port::L; // PL 5 ** 44 ** D44
        case 45:
            return Port::L; // PL 4 ** 45 ** D45
        case 46:
            return Port::L; // PL 3 ** 46 ** D46
        case 47:
            return Port::L; // PL 2 ** 47 ** D47
        case 48:
            return Port::L; // PL 1 ** 48 ** D48
        case 49:
            return Port::L; // PL 0 ** 49 ** D49
        case 50:
            return Port::B; // PB 3 ** 50 ** SPI_MISO
        case 51:
            return Port::B; // PB 2 ** 51 ** SPI_MOSI
        case 52:
            return Port::B; // PB 1 ** 52 ** SPI_SCK
        case 53:
            return Port::B; // PB 0 ** 53 ** SPI_SS
        case 54:
            return Port::F; // PF 0 ** 54 ** A0
        case 55:
            return Port::F; // PF 1 ** 55 ** A1
        case 56:
            return Port::F; // PF 2 ** 56 ** A2
        case 57:
            return Port::F; // PF 3 ** 57 ** A3
        case 58:
            return Port::F; // PF 4 ** 58 ** A4
        case 59:
            return Port::F; // PF 5 ** 59 ** A5
        case 60:
            return Port::F; // PF 6 ** 60 ** A6
        case 61:
            return Port::F; // PF 7 ** 61 ** A7
        case 62:
            return Port::K; // PK 0 ** 62 ** A8
        case 63:
            return Port::K; // PK 1 ** 63 ** A9
        case 64:
            return Port::K; // PK 2 ** 64 ** A10
        case 65:
            return Port::K; // PK 3 ** 65 ** A11
        case 66:
            return Port::K; // PK 4 ** 66 ** A12
        case 67:
            return Port::K; // PK 5 ** 67 ** A13
        case 68:
            return Port::K; // PK 6 ** 68 ** A14
        case 69:
            return Port::K; // PK 7 ** 69 ** A15
        }
    }

    template <uint8_t PIN>
    constexpr inline uint8_t pin_to_mask()
    {
        switch (PIN)
        {
        case 0:
            return _BV(0); // PE 0 ** 0 ** USART0_RX
        case 1:
            return _BV(1); // PE 1 ** 1 ** USART0_TX
        case 2:
            return _BV(4); // PE 4 ** 2 ** PWM2
        case 3:
            return _BV(5); // PE 5 ** 3 ** PWM3
        case 4:
            return _BV(5); // PG 5 ** 4 ** PWM4
        case 5:
            return _BV(3); // PE 3 ** 5 ** PWM5
        case 6:
            return _BV(3); // PH 3 ** 6 ** PWM6
        case 7:
            return _BV(4); // PH 4 ** 7 ** PWM7
        case 8:
            return _BV(5); // PH 5 ** 8 ** PWM8
        case 9:
            return _BV(6); // PH 6 ** 9 ** PWM9
        case 10:
            return _BV(4); // PB 4 ** 10 ** PWM10
        case 11:
            return _BV(5); // PB 5 ** 11 ** PWM11
        case 12:
            return _BV(6); // PB 6 ** 12 ** PWM12
        case 13:
            return _BV(7); // PB 7 ** 13 ** PWM13
        case 14:
            return _BV(1); // PJ 1 ** 14 ** USART3_TX
        case 15:
            return _BV(0); // PJ 0 ** 15 ** USART3_RX
        case 16:
            return _BV(1); // PH 1 ** 16 ** USART2_TX
        case 17:
            return _BV(0); // PH 0 ** 17 ** USART2_RX
        case 18:
            return _BV(3); // PD 3 ** 18 ** USART1_TX
        case 19:
            return _BV(2); // PD 2 ** 19 ** USART1_RX
        case 20:
            return _BV(1); // PD 1 ** 20 ** I2C_SDA
        case 21:
            return _BV(0); // PD 0 ** 21 ** I2C_SCL
        case 22:
            return _BV(0); // PA 0 ** 22 ** D22
        case 23:
            return _BV(1); // PA 1 ** 23 ** D23
        case 24:
            return _BV(2); // PA 2 ** 24 ** D24
        case 25:
            return _BV(3); // PA 3 ** 25 ** D25
        case 26:
            return _BV(4); // PA 4 ** 26 ** D26
        case 27:
            return _BV(5); // PA 5 ** 27 ** D27
        case 28:
            return _BV(6); // PA 6 ** 28 ** D28
        case 29:
            return _BV(7); // PA 7 ** 29 ** D29
        case 30:
            return _BV(7); // PC 7 ** 30 ** D30
        case 31:
            return _BV(6); // PC 6 ** 31 ** D31
        case 32:
            return _BV(5); // PC 5 ** 32 ** D32
        case 33:
            return _BV(4); // PC 4 ** 33 ** D33
        case 34:
            return _BV(3); // PC 3 ** 34 ** D34
        case 35:
            return _BV(2); // PC 2 ** 35 ** D35
        case 36:
            return _BV(1); // PC 1 ** 36 ** D36
        case 37:
            return _BV(0); // PC 0 ** 37 ** D37
        case 38:
            return _BV(7); // PD 7 ** 38 ** D38
        case 39:
            return _BV(2); // PG 2 ** 39 ** D39
        case 40:
            return _BV(1); // PG 1 ** 40 ** D40
        case 41:
            return _BV(0); // PG 0 ** 41 ** D41
        case 42:
            return _BV(7); // PL 7 ** 42 ** D42
        case 43:
            return _BV(6); // PL 6 ** 43 ** D43
        case 44:
            return _BV(5); // PL 5 ** 44 ** D44
        case 45:
            return _BV(4); // PL 4 ** 45 ** D45
        case 46:
            return _BV(3); // PL 3 ** 46 ** D46
        case 47:
            return _BV(2); // PL 2 ** 47 ** D47
        case 48:
            return _BV(1); // PL 1 ** 48 ** D48
        case 49:
            return _BV(0); // PL 0 ** 49 ** D49
        case 50:
            return _BV(3); // PB 3 ** 50 ** SPI_MISO
        case 51:
            return _BV(2); // PB 2 ** 51 ** SPI_MOSI
        case 52:
            return _BV(1); // PB 1 ** 52 ** SPI_SCK
        case 53:
            return _BV(0); // PB 0 ** 53 ** SPI_SS
        case 54:
            return _BV(0); // PF 0 ** 54 ** A0
        case 55:
            return _BV(1); // PF 1 ** 55 ** A1
        case 56:
            return _BV(2); // PF 2 ** 56 ** A2
        case 57:
            return _BV(3); // PF 3 ** 57 ** A3
        case 58:
            return _BV(4); // PF 4 ** 58 ** A4
        case 59:
            return _BV(5); // PF 5 ** 59 ** A5
        case 60:
            return _BV(6); // PF 6 ** 60 ** A6
        case 61:
            return _BV(7); // PF 7 ** 61 ** A7
        case 62:
            return _BV(0); // PK 0 ** 62 ** A8
        case 63:
            return _BV(1); // PK 1 ** 63 ** A9
        case 64:
            return _BV(2); // PK 2 ** 64 ** A10
        case 65:
            return _BV(3); // PK 3 ** 65 ** A11
        case 66:
            return _BV(4); // PK 4 ** 66 ** A12
        case 67:
            return _BV(5); // PK 5 ** 67 ** A13
        case 68:
            return _BV(6); // PK 6 ** 68 ** A14
        case 69:
            return _BV(7); // PK 7 ** 69 ** A15
        }
    }
}

template <uint8_t PIN>
void Pin<PIN>::init()
{
    pinMode(PIN, OUTPUT);
}

template <uint8_t PIN>
void inline __attribute__((always_inline)) Pin<PIN>::pulse()
{
    high();
    low();
}

template <uint8_t PIN>
void inline __attribute__((always_inline)) Pin<PIN>::high()
{
    *internal::port_to_output<internal::pin_to_port<PIN>()>() |= internal::pin_to_mask<PIN>();
}

template <uint8_t PIN>
void inline __attribute__((always_inline)) Pin<PIN>::low()
{
    *internal::port_to_output<internal::pin_to_port<PIN>()>() &= ~internal::pin_to_mask<PIN>();
}

#endif