import argparse
import csv
import os
import sys
import threading
import time
from collections import deque

import pybullet as p
import pybullet_data
import serial

ADC_MAX = 4095
URDF = os.path.join(os.path.dirname(os.path.abspath(__file__)), "brazo.urdf")
TIMEOUT_DATOS = 1.0
PASO_SIM = 1.0 / 240.0
INVERTIR_PINZA = False

MAPA = {"pot1": "joint_1", "pot2": "joint_2", "pot3": "joint_gripper"}
DEDOS = ["joint_dedo_izq", "joint_dedo_der"]


def checksum(texto):
    cs = 0
    for c in texto:
        cs ^= ord(c)
    return cs


def parsear_trama(linea):
    if not linea.startswith("$") or "*" not in linea:
        return None
    carga, cs = linea[1:].rsplit("*", 1)
    if int(cs, 16) != checksum(carga):
        raise ValueError
    campos = carga.split(",")
    if len(campos) != 6 or campos[0] != "B":
        raise ValueError
    seq, p1, p2, p3, pinza = map(int, campos[1:])
    return {"seq": seq, "pot1": p1, "pot2": p2, "pot3": p3, "pinza": pinza}


class LectorSerial(threading.Thread):
    def __init__(self, puerto, baudios):
        super().__init__(daemon=True)
        self.ser = serial.Serial(puerto, baudios, timeout=0.1)
        time.sleep(2.0)
        self.ser.reset_input_buffer()
        self.lock = threading.Lock()
        self.dato, self.t_ultimo = None, 0.0
        self.ok = self.errores = self.perdidos = 0
        self.ultimo_seq = None
        self.marcas = deque(maxlen=60)
        self.activo = True

    def run(self):
        while self.activo:
            try:
                linea = self.ser.readline().decode("ascii", errors="ignore").strip()
            except serial.SerialException:
                print("[ERROR] Puerto serial desconectado")
                break
            try:
                trama = parsear_trama(linea)
            except ValueError:
                with self.lock:
                    self.errores += 1
                continue
            if trama is None:
                continue

            ahora = time.perf_counter()
            with self.lock:
                if self.ultimo_seq is not None:
                    salto = (trama["seq"] - self.ultimo_seq) % 10000
                    self.perdidos += max(0, salto - 1)
                self.ultimo_seq = trama["seq"]
                self.dato, self.t_ultimo = trama, ahora
                self.ok += 1
                self.marcas.append(ahora)

    def leer(self):
        with self.lock:
            return self.dato, self.t_ultimo

    def estadisticas(self):
        with self.lock:
            n = len(self.marcas)
            hz = (n - 1) / (self.marcas[-1] - self.marcas[0]) if n > 1 else 0.0
            return hz, self.ok, self.errores, self.perdidos

    def cerrar(self):
        self.activo = False
        self.ser.close()


class FuenteDemo:
    def __init__(self):
        self.sliders = [
            p.addUserDebugParameter("POT1 joint_1", 0, ADC_MAX, ADC_MAX / 2),
            p.addUserDebugParameter("POT2 joint_2", 0, ADC_MAX, ADC_MAX / 2),
            p.addUserDebugParameter("POT3 gripper", 0, ADC_MAX, 0),
            p.addUserDebugParameter("Pinza", 0, 1, 0),
        ]
        self.seq = 0

    def leer(self):
        self.seq = (self.seq + 1) % 10000
        v = [p.readUserDebugParameter(s) for s in self.sliders]
        dato = {"seq": self.seq, "pot1": int(v[0]), "pot2": int(v[1]),
                "pot3": int(v[2]), "pinza": int(v[3] > 0.5)}
        return dato, time.perf_counter()

    def estadisticas(self):
        return 0.0, self.seq, 0, 0

    def cerrar(self):
        pass


def cargar_robot():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    robot = p.loadURDF(URDF, useFixedBase=True)
    p.resetDebugVisualizerCamera(0.9, 45, -25, [0, 0, 0.3])

    juntas = {}
    for i in range(p.getNumJoints(robot)):
        info = p.getJointInfo(robot, i)
        lo, hi = (info[8], info[9]) if info[8] <= info[9] else (-3.14, 3.14)
        juntas[info[1].decode()] = {
            "indice": i, "min": lo, "max": hi,
            "fuerza": info[10] or 50, "vel": info[11] or 1.0,
        }
    return robot, juntas


def mapear(adc, lo, hi):
    adc = max(0, min(ADC_MAX, adc))
    return lo + (hi - lo) * adc / ADC_MAX


def calcular_objetivos(dato, juntas):
    objetivos = {n: mapear(dato[c], juntas[n]["min"], juntas[n]["max"])
                 for c, n in MAPA.items()}
    cerrada = bool(dato["pinza"]) ^ INVERTIR_PINZA
    for n in DEDOS:
        objetivos[n] = juntas[n]["max"] if cerrada else juntas[n]["min"]
    return objetivos


def mover(robot, junta, objetivo):
    p.setJointMotorControl2(robot, junta["indice"], p.POSITION_CONTROL,
                            targetPosition=objetivo, force=junta["fuerza"],
                            maxVelocity=junta["vel"])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--puerto")
    parser.add_argument("--baudios", type=int, default=115200)
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--log")
    args = parser.parse_args()

    if not os.path.exists(URDF):
        sys.exit("No se encontró brazo.urdf en la carpeta del script.")

    robot, juntas = cargar_robot()
    faltantes = [n for n in [*MAPA.values(), *DEDOS] if n not in juntas]
    if faltantes:
        sys.exit(f"Faltan articulaciones en el URDF: {faltantes}")

    if args.demo:
        fuente = FuenteDemo()
    else:
        if not args.puerto:
            sys.exit("Indica el puerto con --puerto o usa --demo.")
        fuente = LectorSerial(args.puerto, args.baudios)
        fuente.start()

    log = open(args.log, "w", newline="") if args.log else None
    escritor = csv.writer(log) if log else None
    if escritor:
        escritor.writerow(["t_s", "seq", "pot1", "pot2", "pot3", "pinza",
                           "q1_rad", "q2_rad", "gripper_m"])

    id_texto, t_texto, ultimo_seq = -1, 0.0, None
    t0 = time.perf_counter()

    try:
        while p.isConnected():
            dato, t_dato = fuente.leer()
            ahora = time.perf_counter()

            if dato:
                objetivos = calcular_objetivos(dato, juntas)
                for nombre, valor in objetivos.items():
                    mover(robot, juntas[nombre], valor)
                if escritor and dato["seq"] != ultimo_seq:
                    escritor.writerow([f"{ahora - t0:.3f}", dato["seq"], dato["pot1"],
                                       dato["pot2"], dato["pot3"], dato["pinza"],
                                       f"{objetivos['joint_1']:.4f}",
                                       f"{objetivos['joint_2']:.4f}",
                                       f"{objetivos['joint_gripper']:.4f}"])
                ultimo_seq = dato["seq"]

            if ahora - t_texto > 0.25:
                t_texto = ahora
                hz, ok, err, perdidos = fuente.estadisticas()
                if not dato or ahora - t_dato > TIMEOUT_DATOS:
                    texto, color = "SIN DATOS DE LA ESP32", [1, 0, 0]
                else:
                    pinza = "CERRADA" if dato["pinza"] else "ABIERTA"
                    texto = (f"{hz:4.1f} Hz | OK {ok} | err {err} | perdidos {perdidos} | "
                             f"edad {(ahora - t_dato) * 1000:3.0f} ms | pinza {pinza}")
                    color = [0, 0.5, 0]
                id_texto = p.addUserDebugText(texto, [0, 0, 0.75], color, 1.2,
                                              replaceItemUniqueId=id_texto)

            p.stepSimulation()
            time.sleep(PASO_SIM)
    except KeyboardInterrupt:
        pass
    finally:
        fuente.cerrar()
        if log:
            log.close()
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    main()
