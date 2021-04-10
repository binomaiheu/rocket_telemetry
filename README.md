# Waterrocket telemetry project

## Starting & stopping a run

Publish a "1" to the the "run" topic on the mqtt broker to trigger the device to start publishing telemetry
data to the broker. Publish a "0" to stop the run. When starting a run, the pressure sensor is polled 10 times
to get and offset calibration for the altitude measurement via the BMP280 sensor. Also the timer is reset, to 
have the time in milliseconds since the start of the run. No timer overflow handling is done. 

## Configuration

You can change the accelerometer readout range  by publishing 

```json
{
  "mma8452q_scale": 4
}
```

to the /config topic of the sensor device in the broker. The values can be 2, 4 or 8, see : 


## Payload

The status payload is published every 10 seconds regardless of the run status

```json
{
  "wifiStatus": 3,
  "macAddress": "f0:c8:ea:a4:ae:30",
  "ipAddress": "192.168.1.104",
  "ssid": "network",
  "rssi": -64,
  "battery_volt": 4.21,
  "mqttStatus": 1
}
```

The measurement payloads are published when the run is active. 

```json
{
  "timestamp": 1000911,
  "temp_C": 27.81,
  "pressure_Pa": 101012.98,
  "altitude_m": 1.88,
  "max_altitude_m": 4.08,
  "ax": -0.0703,
  "ay": 0.0391,
  "az": 0.9922
}
```
