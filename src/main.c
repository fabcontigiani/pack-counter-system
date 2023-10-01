#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

// Pines
// Decodificador BCD a 7-seg: PB0-PB3
#define P1 PORTD0     // Pulsador 1
#define P2 PORTD1     // Pulsador 2
#define P3 PORTD2     // Pulsador 3
#define ENABLE PORTD6 // LED Indicador de Conteo Habilitado
#define ALARMA PORTD7 // Alarma (Transistor Q4)
#define Q1 PORTC0     // Transistor Q1
#define Q2 PORTC1     // Transistor Q2
#define Q3 PORTC2     // Transistor Q3

#define BOUNCE_DELAY 10
#define TIMER0_CTC_TOP 156   // * 1024 * (1/16MHz) = 9.984ms ~= 10ms
#define TIMER1_CTC_TOP 31250 // * 1024 * (1/16MHz) = 2s

uint16_t threshold;
uint8_t enable = 1;
uint8_t flag = 1;
volatile uint16_t count = 50;
volatile uint8_t transistor = Q1;
volatile uint8_t timer1_counter = 0;
uint8_t P1State;
uint8_t lastP1State = 1;
uint8_t P2State;
uint8_t lastP2State = 1;
uint8_t P3State;
uint8_t lastP3State = 1;

int main(void) {
    DDRD =
        (1 << ALARMA) | (1 << ENABLE); // Puerto D: Pulsadores (entrada), alarma
                                       // y led inidador de enable (salida)
    PORTD = (1 << P1) | (1 << P2) | (1 << P3) |
            (1 << ENABLE); // Pull-ups internos. Enable en alto por defecto
    DDRC = 0xFF;           // Puerto C: Transistores (modo salida)
    DDRB = 0xFF;           // Puerto B: Displays (modo salida)

    // Timer 0: Multiplexado de Displays
    TCCR0A = (1 << WGM01); // Modo CTC
    OCR0A = TIMER0_CTC_TOP;
    TIMSK0 = (1 << OCIE0A); // Interrupción por igualación TCNT0 == OCR0A
    TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler 1024. Inicia el conteo
    sei();                              // Setea bit global de interupciones

    // Timer 1 : Apagado de alarma tras 10s
    TCCR1B = (1 << WGM12); // Modo CTC
    OCR1A = TIMER1_CTC_TOP;
    TIMSK1 = (1 << OCIE1A); // Interrupción por igualación TCNT1 == OCR1A

    // Modo configuración
    // P1: Configura valor umbral.
    // P2: Habilita el conteo (sale del modo configuración).
    // P3: No tiene efecto.
    // Display: Muestra el incremento del umbral.
    do {
        P1State = (PIND & (1 << P1));
        if (P1State != lastP1State) { // cambió estado del botón
            if (!P1State) {           // botón pasó de alto a bajo
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P1))) { // botón sigue en bajo tras retardo
                    count += 1;
                    if (count > 400)
                        count = 50;
                }
            }
        }
        lastP1State = P1State;

        P2State = (PIND & (1 << P2));
        if (P2State != lastP2State) {
            if (!P2State) {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P2)))
                    flag = 0;
            }
        }
        lastP2State = P2State;
    } while (flag);

    threshold = count;
    count = 0;

    while (1) {
        // Modo conteo de packs
        // P1: Resetea cantidad contada.
        // P2: Detiene y reanuda el conteo.
        // P3: Incrementa conteo (simula detector de packs).
        // Display: Indica la cantidad de packs contados (no resetea al llegar
        // al umbral).
        P1State = (PIND & (1 << P1));
        if (P1State != lastP1State) {
            if (!P1State) {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P1)))
                    count = 0;
            }
        }
        lastP1State = P1State;

        P2State = (PIND & (1 << P2));
        if (P2State != lastP2State) {
            if (!P2State) {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P2))) {
                    enable ^= 1;
                    PORTD ^= (1 << ENABLE);
                }
            }
        }
        lastP2State = P2State;

        P3State = (PIND & (1 << P3));
        if ((P3State != lastP3State) && enable) {
            if (!P3State) {
                _delay_ms(BOUNCE_DELAY);
                if (!(PIND & (1 << P3))) {
                    count += 1;
                    if (count > 999)
                        count = 0;
                }
            }
        }
        lastP3State = P3State;

        if ((count >= threshold) && !(PIND & (1 << ALARMA))) {
            // si contador por encima de umbral y alarma no está encendida
            // encender alarma por 10 segundos
            PORTD |= (1 << ALARMA);
            TCCR1B |=
                (1 << CS12) | (1 << CS10); // Prescaler 1024. Inicia el conteo
        }
    }

    return 0;
}

ISR(TIMER0_COMPA_vect) {
    switch (transistor) {
    case Q1:          // se está mostrando el display de las unidades
        PORTC = 0x00; // se apagan los displays
        PORTB = (count / 10) % 10; // se escribe en el puerto
        PORTC = (1 << Q2);         // se enciende el display de las decenas
        transistor = Q2;
        break;

    case Q2: // se está mostrando el display de las decenas
        PORTC = 0x00;
        PORTB = count / 100;
        PORTC = (1 << Q3);
        transistor = Q3;
        break;

    case Q3: // se está mostrando el display de las centenas
        PORTC = 0x00;
        PORTB = count % 10;
        PORTC = (1 << Q1);
        transistor = Q1;
        break;
    }
}

ISR(TIMER1_COMPA_vect) {
    timer1_counter++;
    if (timer1_counter >= 5) {
        PORTD &= ~(1 << ALARMA);               // Apaga alarma
        TCCR1B &= ~(1 << CS12) & ~(1 << CS10); // Detiene conteo
        timer1_counter = 0;
    }
}