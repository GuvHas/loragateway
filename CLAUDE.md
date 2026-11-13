# loragateway

ESP32 (TTGO LoRa32 v2.1) firmware that receives sensor readings (binary frame or
JSON) over LoRa
and publishes them to MQTT with Home Assistant discovery. The sensor nodes run
the companion firmware in [GuvHas/loratemp](https://github.com/GuvHas/loratemp).

## Commands

```sh
pio test -e native            # unit tests, no hardware needed
pio run -e ttgo-lora32-v21    # firmware build
```

CI runs both on every push and pull request. Run both before pushing.

Send `pio run` output to a log file and read the end of it. Piping it into
`head` closes the pipe early and the build hangs.

The ESP32 platform is pinned to 8.0.0 (Arduino core 3.x, ESP-IDF 5) in
`platformio.ini`; before that it was 7.1.3 (Arduino core 2.0.17). Core 3.x
changed the task watchdog API and starts the watchdog before `setup()`, so
`setupWatchdog()` in `main.cpp` reconfigures it rather than initialising it.
Core 3.x also outgrew the default 1.25 MB app partition, so the firmware uses
`min_spiffs.csv` (two 1.9 MB OTA slots, NVS unchanged, no filesystem in use).
A partition table change has to be flashed over USB.
A platform bump is its own change, and the owner tests it on the board.
`WiFiManager` is still taken from the head of its git repository.

## Layout

| Path | What it holds |
| --- | --- |
| `include/payload_parser.h`, `src/payload_parser.cpp` | Pure functions: payload parsing, allowlist, routing, sanitizers, MQTT and discovery message builders. No Arduino headers. |
| `include/hal.h` | Interfaces for WiFi, LoRa, MQTT, storage, display and clock. |
| `include/hal_esp32.h`, `src/hal_esp32.cpp` | Thin ESP32 implementations of those interfaces. |
| `include/orchestrator.h`, `src/orchestrator.cpp` | `GatewayOrchestrator`: packet ingestion, deduplication, discovery, the store-and-forward queue, reconnect logic. Uses only `hal.h`. |
| `src/main.cpp` | Wiring, config portal, web pages, OTA, watchdog. |
| `scripts/inject_git_version.py` | Injects the git short hash as `GATEWAY_FW_VERSION`. |
| `test/test_parser`, `test/test_orchestrator` | Unity tests. `fakes.h` has in-memory fakes for every `hal.h` interface. |

The `native` environment does not compile `main.cpp` or `hal_esp32.cpp`, so
logic that needs a test belongs in `payload_parser` or `orchestrator`.

## How changes are made

1. Branch from `main`.
2. Write or update the native test first, then implement.
3. Run both commands above.
4. Open a pull request against `main`.
5. Codex reviews every pull request automatically. Treat each finding as a bug
   report: check it against the code, fix it with a test, reply on the thread
   and resolve it.
6. The owner merges, or asks for the merge, once CI is green.

Behaviour that depends on real hardware, the broker or Home Assistant cannot be
verified from this repository. Say so in the pull request and leave it as a
manual check for the owner.

## Payload contract with loratemp

```json
{"id":"GarageTemp","t":21.5,"h":55.2,"v":3.90,"boot":7,"seq":4,"lb":0,"err":"none","sw":"fde884a"}
```

- `t`, `h` and `v` are numbers or `null`. `lb` is 0 or 1. `err` is a string.
- `boot` counts wake-ups since the node last lost power. The node increments it
  before sending, so `boot` is 1 on the first packet after a flash or battery
  swap.
- `seq` counts transmissions.
- `sw` is the node's firmware version. The node sends it until one
  transmission carrying it succeeds, on its first 3 sends after power-up, and
  once a day (loratemp `shouldReportVersion()`). Repeats of an unchanged
  version are no-ops here.
- A field with the wrong type rejects the whole payload. Unknown fields are
  ignored.

### Binary frame v1

Nodes may send a compact binary frame instead of JSON (24 bytes instead of
~83 for a typical reading, about 206 ms instead of 472 ms on air at SF9).
`parseSensorPacket()` picks the format from the first byte: `0xC1` is a
binary frame, anything else goes to the JSON parser. Both decode into the
same `SensorReading`, so MQTT and Home Assistant see no difference. Both
formats stay accepted, because nodes are reflashed one at a time.

Little-endian. N is the id length.

| Offset | Size | Field | Encoding |
| --- | --- | --- | --- |
| 0 | 1 | Magic and version | `0xC1` |
| 1 | 1 | Flags | bit 0 `lb`, bit 1 sensor error (`err` `"dht"`), bit 2 `sw` present; bits 3 to 7 reserved, ignored |
| 2 | 1 | N | id length, at least 1 |
| 3 | N | `id` | ASCII, sanitized like the JSON id |
| 3+N | 3 | `boot` | uint24 |
| 6+N | 3 | `seq` | uint24 |
| 9+N | 2 | `t` | int16, 0.1 °C; `0x8000` is null |
| 11+N | 2 | `h` | uint16, 0.1 %; `0xFFFF` is null |
| 13+N | 1 | `v` | uint8, (mV − 2000) / 10 |
| 14+N | 1+M | `sw` | only with flag bit 2: length M (at least 1), then ASCII |

The length must match the header exactly, or the frame is rejected
(`ParseError::BadFrame`) and published on `<base>` as lowercase hex. The hex
vectors in `test/test_parser/test_parser.cpp` are the golden vectors that
loratemp's encoder must reproduce byte for byte; change them together.

Nodes have no WiFi and no OTA, so a node in the field runs old firmware until
someone reflashes it by hand. Contract changes must keep old nodes working, and
the gateway change ships first.

Radio settings must match on both sides: 868 MHz, spreading factor 9, CRC on.

## MQTT topics

`<base>` is the configured base topic, `lora/incoming` by default.

| Topic | Retained | Content |
| --- | --- | --- |
| `<base>/<node id, lowercased>` | yes | Node reading plus `rssi` |
| `<base>` | no | Raw bytes of a JSON packet that failed to parse, or lowercase hex of a bad binary frame |
| `<base>/gateway/status` | yes | `online` or `offline` (last will). Availability topic for every entity. |
| `<base>/gateway/state` | yes | Gateway diagnostics JSON |
| `<base>/gateway/command/restart` | no | `PRESS` restarts the gateway |
| `homeassistant/status` | (subscribed) | `online` from Home Assistant resends all discovery |
| `homeassistant/<component>/lora_<slug>_<suffix>/config` | yes | Node discovery, 8 entities per node |
| `homeassistant/<component>/<gateway slug>_<suffix>/config` | yes | Gateway discovery |

## Rules that exist because something broke

- **Node ids are untrusted.** LoRa has no authentication. Ids go through
  `sanitizeMqttTopicSegment()` before any topic, `haSafeSlug()` before any
  discovery topic or unique id, and `htmlEscape()` / `urlEncodeComponent()`
  before any web page.
- **ArduinoJson drops fields silently when a document is full.** Discovery
  documents use `kDiscoveryJsonCapacity` (700). When a field is added, the test
  must deserialize the payload and assert on that field.
- **Leaving a key out does not clear it in Home Assistant.** Home Assistant
  merges device info across the configs that share a device id. Node discovery
  therefore sends `"sw": null` when the version is unknown.
- **Node state is retained.** Discovery and the first reading go out in one
  burst, and Home Assistant misses a non-retained reading that arrives before
  it has subscribed. `removeNode()` publishes an empty retained payload to
  clear it.
- **Deduplication resets on a higher `boot`, or on `boot == 1`.** The
  `(boot, seq)` watermark lives in RAM. A reflashed node restarts at 1, below
  the watermark, and was dropped until the gateway restarted. Do not widen the
  reset to any change in `boot`: that lets two old packets be replayed in
  turn indefinitely.
- **`sw` is captured before the pending check and before deduplication,** and
  persisted under the `swver` Preferences key. A node sends it on only a few
  packets, often before it is approved.
- **Do not overwrite `pendingDiscoveryCount_` while a batch is queued.** A
  version change during a batch goes into `pendingRediscovery_` and is
  republished when the batch completes.
- **Discovery bookkeeping is RAM only.** After a restart every node is
  rediscovered on its next packet.
- **`tick()` must return in bounded time.** It drains at most 20 packets, and
  the offline queue holds at most 20 messages, dropping the oldest.
- **LoRa receive runs in the DIO0 interrupt.** That code must be `IRAM_ATTR`,
  must not allocate and must not log. `LoRa.onReceive()` takes a plain function
  pointer, hence the static trampoline. PubSubClient's callback is a
  `std::function` on ESP32 and needs none.
- **The restart handler clears the retained command first.** A retained
  `PRESS` would otherwise restart the gateway on every reconnect.
- **Preferences key names are limited to 15 characters.**
- **Publish only when `online()`, never on `mqtt.connected()` alone.**
  PubSubClient keeps reporting connected over a dead socket after WiFi drops,
  and each write into it can block for 10 s, enough to trip the 30 s watchdog
  during an AP reboot. The orchestrator closes that session when WiFi drops.
- **The setup portal must time out.** WiFiManager switches the WiFi station
  off when its portal starts without a connection, and `WiFi.reconnect()`
  cannot switch it back on. With no timeout, a gateway that booted during a
  network outage could stay offline until power-cycled. While the portal is open
  WiFi recovery is paused; after it closes, recovery backs off from 10 s to
  60 s, makes every third attempt `IWifiRadio::hardReset()`, and requests a
  restart after 10 minutes offline.
- **Boot-time WiFi connect must finish inside the watchdog.** WiFiManager's
  default connect timeout falls back to 60 s; it is set to 15 s.

## When a node does not show up in Home Assistant

Check in this order:

1. `http://<gateway-ip>/devices`: is the node pending or approved? Pending
   nodes publish nothing.
2. `<base>/gateway/state`: does `packets_rx` rise when the node transmits? If
   not, the radio is not receiving it.
3. `<base>/<node id>`: is there a reading? If `packets_rx` rises and there is
   none, the packet was deduplicated, or failed to parse and went to `<base>`.
4. `<base>/gateway/status`: is it `online`?
5. `homeassistant/sensor/lora_<slug>_*/config`: are the discovery configs
   there, and does `stat_t` match the state topic?

## Known gaps

- `removeNode()` does not clear the node's retained discovery configs, so the
  device stays in Home Assistant.
- `removeNode()` does not clear the node's deduplication watermark.
- Nothing is sent from the gateway to a node. `gatewayCommandTopic()` reserves
  the topic shape for it.
