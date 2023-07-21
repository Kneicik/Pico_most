import network
import socket
from time import sleep
from picozero import pico_temp_sensor, pico_led
import machine
from machine import Pin, PWM
import math

# ssid = "RoboLAB - Goscie"
# password = "FWEPSDL_2022"
ssid = "voron"
password = "12345678"

endstop_l = Pin(15, Pin.IN, Pin.PULL_DOWN)
endstop_r = Pin(14, Pin.IN, Pin.PULL_DOWN)
emergency = Pin(13, Pin.IN, Pin.PULL_DOWN)

def connect():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)
    while wlan.isconnected() == False:
        print("Waiting for connection...")
        pico_led.on()
        sleep(0.5)
        pico_led.off()
        sleep(0.5)
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    pico_led.on()
    return ip
    
def open_socket(ip):
    address = (ip, 80)
    connection = socket.socket()
    connection.bind(address)
    connection.listen(1)
    return connection

def webpage(temperature, state):
    html = f"""
            <!DOCTYPE html>
            <html>
            <form action="./mostdown">
            <input type="submit" value="Most down" />
            </form>
            <form action="./mostup">
            <input type="submit" value="Most up" />
            </form>
            <form action="./motoroff">
            <input type="submit" value="Motor off" />
            </form>
            </body>
            </html>
            """
    return str(html)


motor_l = PWM(Pin(16))
motor_r = PWM(Pin(17))
motor_l.freq(1000)
motor_r.freq(1000)

def serve(connection):
    state = "OFF"
    temperature = 0
    while True:
        client = connection.accept()[0]
        request = client.recv(1024)
        request = str(request)
        try:
            request = request.split()[1]
        except IndexError:
            pass
        if request == '/mostdown?':
            while endstop_l.value() == 0:
                motor_l.duty_u16(65000)
                if request == '/motoroff?':
                    motor_l.duty_u16(0)
            motor_l.duty_u16(0)
        elif request == '/mostup?':
            while endstop_r.value() == 0:
                motor_r.duty_u16(65000)
                if request == '/motoroff?':
                    motor_r.duty_u16(0)
            motor_r.duty_u16(0)
        elif request =='/motoroff?':
            motor_l.duty_u16(0)
            motor_r.duty_u16(0)
        html = webpage(temperature, state)
        client.send(html)
        client.close()

        
try:
    ip = connect()
    connection = open_socket(ip)
    serve(connection)
except KeyboardInterrupt:
    machine.reset()
    
#dodać zabezpieczenie grzyba i na stronie piny 16 i 17 są do pwm piny 15,14,13 od prazycisków