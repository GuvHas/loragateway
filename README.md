# LoRa to MQTT Gateway

[![Build Firmware Artifact](https://github.com/GuvHas/loragateway/actions/workflows/build-artifact.yml/badge.svg)](https://github.com/GuvHas/loragateway/actions/workflows/build-artifact.yml)

An ESP32 (TTGO LoRa32) gateway that receives sensor readings (binary frame or JSON) over LoRa
and publishes them to MQTT with Home Assistant MQTT-discovery, so each node
shows up automatically as a device with its own sensors.

## Download Latest Firmware

Every push to `main` automatically builds the firmware and attaches it as a
downloadable artifact — no local PlatformIO setup needed to get a flashable
binary. The badge above shows whether the latest build succeeded.

**This is an application-image update, not a full/blank-chip flash.** The
artifact contains only `firmware.bin` — no bootloader or partition-table
binaries — so it only works on a board that already has this project's
bootloader and partition layout on it (e.g. one that's ever been flashed via
`pio run -e ttgo-lora32-v21 -t upload`, which writes all three at first
flash). Setting up a brand-new/erased board still needs that full PlatformIO
upload once; after that, this artifact can be used for updates.

1. Open the [`build-artifact.yml` workflow runs page](https://github.com/GuvHas/loragateway/actions/workflows/build-artifact.yml).
2. Click the topmost run (it's the latest build off `main`; a green check
   means it succeeded).
3. Scroll down to the **Artifacts** section at the bottom of the run page.
4. Download the `firmware-<commit-sha>` zip and unzip it — it contains
   `firmware.bin`. Flash it at the application offset (`0x10000` on this
   board) with `esptool.py --chip esp32 write_flash 0x10000 firmware.bin`,
   or drop it in place of `.pio/build/ttgo-lora32-v21/firmware.bin` and run
   the PlatformIO upload tools.

Note: downloading workflow artifacts requires being signed in to GitHub, since
that's a GitHub-wide restriction on Actions artifacts, not something this repo
can turn off.

> **Upgrading from a build before the move to platform 8.0.0 (#27)?** The
> firmware now uses the `min_spiffs.csv` partition table (two 1.9 MB OTA
> slots), because Arduino core 3.x outgrew the default app partition. A
> partition table change can't be applied by OTA or by flashing
> `firmware.bin` alone, so flash that one update over USB with
> `pio run -e ttgo-lora32-v21 -t upload`. Later updates can use the artifact
> or OTA again.

## First setup

1. Flash the board over USB (see above).
2. On first boot, with no saved WiFi, the gateway opens a setup portal as the
   access point `LoRaGateway-Setup`. Connect to it and enter your WiFi and
   MQTT broker details.
3. The portal closes after 5 minutes (it stays open while a phone is
   connected), and the gateway keeps trying to join the saved network on its
   own. A gateway that boots during a power or network outage therefore comes
   back by itself once the router is up, without a power cycle.
4. Open `http://<gateway-ip>/devices` and approve your sensor nodes. They
   then appear in Home Assistant through MQTT discovery.

## Supported packets

The gateway accepts two packet formats from the
[loratemp](https://github.com/GuvHas/loratemp) nodes and treats them the same:

- **Binary frame v1** (default on loratemp since #18): 24 bytes for a typical
  reading, about 206 ms on air at SF9.
- **JSON**, e.g.
  `{"id":"GarageTemp","t":21.5,"h":55.2,"v":3.90,"boot":7,"seq":4,"lb":0,"err":"none","sw":"fde884a"}`
  (about 83 bytes, 472 ms on air).

The first byte decides: `0xC1` is a binary frame, anything else is parsed as
JSON. Both formats stay supported, so nodes can be reflashed one at a time.
The byte layout is in [CLAUDE.md](CLAUDE.md#binary-frame-v1). Radio settings
must match the nodes: 868 MHz, spreading factor 9, CRC on.

## Home Assistant entities

<!--
  SCREENSHOT PLACEHOLDER: insert a new screenshot of the Home Assistant
  device page for a node here, showing the sensors and the diagnostic
  entities (including SNR and Packet Loss, added in #30). Upload the image
  by dragging it into a GitHub comment or edit box, then replace the line
  below with the generated <img> tag or an ![...](...) link. The two older
  screenshots further down can be removed once this one is in.

  ![Home Assistant sensors for a LoRa node](docs/ha-node-sensors.png)
-->

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
SNR (dB, of the node's last packet)
Packet Loss (%, of the node's last 100 expected packets, from gaps in
`seq`; starts over when the node reboots)

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
- Uptime
- A **Restart** button

The gateway state JSON on `<base>/gateway/state` (base topic
`lora/incoming` by default) also carries `wifi_reconnects` (how many times
WiFi has been recovered since boot) and `wifi_disc_reason` (the ESP-IDF
reason code of the last WiFi disconnect). They have no Home Assistant entity
yet; read them with an MQTT client or a template sensor when chasing WiFi
drops.

## Resilience

- **Store-and-forward MQTT queue.** If the MQTT broker is unreachable, LoRa
  readings (and Home Assistant discovery messages) are queued instead of
  dropped, then flushed once the broker is reachable again. The queue is
  bounded (32 messages); if it fills while still offline, the oldest entry
  is dropped to make room for the newest.
- **Non-blocking WiFi + MQTT reconnect.** Both connections are retried on
  their own backoff timers without blocking LoRa packet reception, so a
  dropped router/AP or broker doesn't stall the gateway. WiFi retries back
  off from 10 s to 60 s, every third attempt fully resets the WiFi radio, and
  after 10 minutes offline the gateway restarts itself. Nothing is published
  while WiFi is down, so a dead MQTT socket can't block long enough to trip
  the 30 s watchdog.
- **Setup portal timeout.** If the gateway boots without WiFi, the setup
  portal closes after 5 minutes and normal WiFi recovery takes over, so the
  gateway doesn't stay offline until someone power-cycles it.
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

This is a [PlatformIO](https://platformio.org/) project. The ESP32 platform is
pinned to `espressif32 @ 8.0.0` (Arduino core 3.x, ESP-IDF 5), the same
version as loratemp.

```sh
# Native unit tests (no hardware required)
pio test -e native

# Build firmware for the TTGO LoRa32 v2.1
pio run -e ttgo-lora32-v21
```

CI runs both on every push and pull request. [CLAUDE.md](CLAUDE.md) has the
full payload contract, MQTT topic list and a troubleshooting checklist for a
node that doesn't show up in Home Assistant.

Sensor repo: https://github.com/GuvHas/loratemp
