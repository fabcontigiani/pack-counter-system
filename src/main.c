// Pines
// Pulsador 1: PD0
// Pulsador 2: PD1
// Pulsador 3: PD2
// Alarma: PD7
// Transistor Q1: PC0
// Transistor Q2: PC1
// Transistor Q3: PC2
// Display: PB0-PB3

#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#define P1 PORTD0
#define P2 PORTD1
#define P3 PORTD2
#define ENABLE PORTD6
#define ALARMA PORTD7
#define Q1 PORTC0
#define Q2 PORTC1
#define Q3 PORTC2

#define BOUNCE_DELAY 10
#define TIMER0_CTC_TOP 156   // 10ms
#define TIMER1_CTC_TOP 31250 // 2s

uint16_t threshold;
uint8_t enable = 1;
volatile uint16_t count = 50;
volatile uint8_t transistor = Q1;
volatile uint8_t timer1_counter = 0;

int main(void) {
    DDRD = (1 << ALARMA) | (1 << ENABLE); // Puerto D: Pulsadores (entrada),
    // alarma y led inidador de enable (salida)
    PORTD =
        (1 << P1) | (1 << P2) | (1 << P3) | (1 << ENABLE); // Pull-ups internos
    DDRC = 0xFF; // Puerto C: Transistores (modo salida)
    DDRB = 0xFF; // Puerto B: Displays (modo salida)

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
        if (!(PIND & (1 << P1))) {
            _delay_ms(BOUNCE_DELAY);
            if (!(PIND & (1 << P1))) {
                count += 1;
            }
        }
        if (count > 400)
            count = 50;
    } while ((PIND & (1 << P2)));

    threshold = count;
    count = 0;

    while (1) {
        // Modo conteo de packs
        // P1: Resetea cantidad contada.
        // P2: Detiene y reanuda el conteo.
        // P3: Incrementa conteo (simula detector de packs).
        // Display: Indica la cantidad de packs contados (no resetea al llegar
        // al umbral).
        if (!(PIND & (1 << P1))) {
            _delay_ms(BOUNCE_DELAY);
            if (!(PIND & (1 << P1)))
                count = 0;
        }
        if (!(PIND & (1 << P2))) {
            _delay_ms(BOUNCE_DELAY);
            if (!(PIND & (1 << P2)))
                enable ^= 1;
            PORTD ^= (1 << ENABLE);
        }
        if (!(PIND & (1 << P3)) && enable) {
            _delay_ms(BOUNCE_DELAY);
            if (!(PIND & (1 << P3)))
                count += 1;
        if ((count >= threshold) && !(PIND & (1 << ALARMA))) {
            PORTD |= (1 << ALARMA);
            TCCR1B |=
                (1 << CS12) | (1 << CS10); // Prescaler 1024. Inicia el conteo
        }
        if (count > 999)
            count = 0;
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
        PORTD &= ~(1 << ALARMA);
        TCCR1B &= ~(1 << CS12) & ~(1 << CS11) &
                  ~(1 << CS10); // Limpia prescaler. Detiene conteo
        TCNT1 = 0x0000;         // Limpia contador
        timer1_counter = 0;
    }
}