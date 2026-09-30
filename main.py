from machine import ADC, Pin
import sys
import time

PINES_POT = (34, 35, 32)
PIN_BOTON = 25
PIN_LED = 2

PERIODO_MS = 50
N_MUESTRAS = 16
ZONA_MUERTA = 15
ANTIRREBOTE_MS = 200
ADC_MAX = 4095


def crear_adc(pin):
    adc = ADC(Pin(pin))
    adc.atten(ADC.ATTN_11DB)
    return adc


def leer_adc(adc):
    total = 0
    for _ in range(N_MUESTRAS):
        total += adc.read_u16() >> 4
    return total // N_MUESTRAS


def filtrar(nuevo, anterior):
    if nuevo <= ZONA_MUERTA:
        return 0
    if nuevo >= ADC_MAX - ZONA_MUERTA:
        return ADC_MAX
    return nuevo if abs(nuevo - anterior) > ZONA_MUERTA else anterior


def checksum(texto):
    cs = 0
    for c in texto:
        cs ^= ord(c)
    return cs


pots = [crear_adc(p) for p in PINES_POT]
boton = Pin(PIN_BOTON, Pin.IN, Pin.PULL_UP)
led = Pin(PIN_LED, Pin.OUT)


def main():
    valores = [leer_adc(a) for a in pots]
    pinza = 0
    boton_ant = boton.value()
    t_boton = time.ticks_ms()
    seq = 0
    proximo = time.ticks_ms()

    while True:
        for i, adc in enumerate(pots):
            valores[i] = filtrar(leer_adc(adc), valores[i])

        estado = boton.value()
        ahora = time.ticks_ms()
        if boton_ant == 1 and estado == 0 and time.ticks_diff(ahora, t_boton) > ANTIRREBOTE_MS:
            pinza ^= 1
            led.value(pinza)
            t_boton = ahora
        boton_ant = estado

        carga = "B,{},{},{},{},{}".format(seq, *valores, pinza)
        sys.stdout.write("${}*{:02X}\n".format(carga, checksum(carga)))
        seq = (seq + 1) % 10000

        proximo = time.ticks_add(proximo, PERIODO_MS)
        espera = time.ticks_diff(proximo, time.ticks_ms())
        if espera > 0:
            time.sleep_ms(espera)
        else:
            proximo = time.ticks_ms()


try:
    main()
except KeyboardInterrupt:
    led.value(0)
  
