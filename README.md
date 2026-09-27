# LoRa to MQTT Gateway

An ESP32 (TTGO LoRa32) gateway that receives JSON sensor readings over LoRa
and publishes them to MQTT with Home Assistant MQTT-discovery, so each node
shows up automatically as a device with its own sensors.

### Sensors created (per node)
Battery
Humidity
Low Battery
Signal
Temperature
<img width="330" height="332" alt="sensors" src="https://github.com/user-attachments/assets/e36319d5-3c55-4dd2-af00-ee542f39a500" />

Temperature/Humidity go `unknown` in Home Assistant (rather than showing a
stale or garbage value) whenever the sensor node reports a failed DHT read.

### Diagnostic (per node)
Boot Count
Error
Sequence

<img width="334" height="233" alt="diagnostic" src="https://github.com/user-attachments/assets/9dda212a-3eea-4a78-8e33-3c9b1c14ef37" />

### Gateway diagnostics
The gateway also publishes its own diagnostic device, covering:
- WiFi Signal
- Free Memory
- Packets Received
- Queue Depth — messages currently held because MQTT is unreachable (see
  "Resilience" below)
- Packets Dropped — messages permanently discarded because that queue
  filled up while still offline

## Resilience

- **Store-and-forward MQTT queue.** If the MQTT broker is unreachable, LoRa
  readings (and Home Assistant discovery messages) are queued instead of
  dropped, then flushed once the broker is reachable again. The queue is
  bounded (20 messages); if it fills while still offline, the oldest entry
  is dropped to make room for the newest.
- **Non-blocking WiFi + MQTT reconnect.** Both connections are retried on
  their own backoff timers without blocking LoRa packet reception, so a
  dropped router/AP or broker doesn't stall the gateway.
- **Untrusted-input hardening.** LoRa has no authentication, so a node id
  is treated as attacker-controlled: it's sanitized before use in MQTT
  topics, escaped before rendering on the device-management web page, and
  restricted to Home Assistant's stricter discovery-topic character set
  before being used in a discovery topic/unique ID — with a stable,
  collision-resistant fallback for ids containing characters outside that
  set (e.g. from a rare over-the-air bit error).
- **Device management.** New nodes appear as "pending" at
  `http://<gateway-ip>/devices` until approved, so only known nodes get
  published and auto-discovered.

## Architecture

The gateway's LoRa/MQTT/WiFi/allowlist logic is separated from the ESP32
hardware so it can be unit tested off-target:

- `include/payload_parser.h` / `src/payload_parser.cpp` — pure payload
  parsing, routing decisions, the node allowlist, and MQTT/HA message
  builders. No Arduino dependency.
- `include/hal.h` / `include/hal_esp32.h` / `src/hal_esp32.cpp` — hardware
  interfaces (LoRa, MQTT, WiFi, display, storage, clock) and their thin
  ESP32 implementations.
- `include/orchestrator.h` / `src/orchestrator.cpp` — `GatewayOrchestrator`,
  which drives LoRa ingestion, routing, publishing, the store-and-forward
  queue, and WiFi/MQTT reconnect via the HAL interfaces above.
- `src/main.cpp` — wires the concrete ESP32 adapters together; `loop()`
  mostly just feeds the watchdog and calls `orchestrator.tick()`.

## Building & testing

This is a [PlatformIO](https://platformio.org/) project.

```sh
# Native unit tests (no hardware required)
pio test -e native

# Build firmware for the TTGO LoRa32 v2.1
pio run -e ttgo-lora32-v21
```

Sensor repo: https://github.com/GuvHas/loratemp
