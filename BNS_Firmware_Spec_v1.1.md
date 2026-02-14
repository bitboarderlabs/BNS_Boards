# BNS Board Family — Firmware Specification

**Baby & Mama Boards**

| | |
|---|---|
| **Document Version** | 1.1 |
| **Date** | February 2026 |
| **Author** | Bitboarder Labs |
| **MCU Target** | STM32F407 (all board variants) |
| **Status** | Decisions Finalized — Ready for Implementation |

### Revision History

- **v1.0** — Initial draft specification
- **v1.1** — Resolved all open questions. Removed Papa board (cancelled). Finalized MQTT, flash, socket allocation, and analog publishing decisions.

---

## 1. Overview

The BNS board family is a modular industrial I/O control platform built on the STM32F407. Two board variants — Baby and Mama — share a common firmware codebase, with hardware-specific behavior determined at runtime via a hardwired board ID and compile-time preprocessor defines. The firmware provides digital and analog I/O control accessible through multiple communication interfaces: RS-422/485 serial, Ethernet (TCP sockets via W5500), USB CDC, a web-based UI, Modbus TCP, and MQTT.

The platform extends the proven architecture of the HVAC RemCon board (FacCon project), adding: variable I/O counts per board variant, an 8-bit parallel I/O bus for Mama boards, a user-programmable sequencer engine (up to 4 concurrent apps stored in on-chip flash), optional LCD display support (Mama only), and comprehensive user-configurable MQTT integration.

> ❌ **REMOVED:** Papa board has been cancelled and will not be manufactured. All Papa-specific code paths, I/O counts, and board ID handling should be removed or stubbed with compile guards during implementation. The `BOARD_TYPE_PAPA` enum value is retained as reserved but unused.

### 1.1 Board Variants

| Feature | Baby | Mama | I/O Method | Notes |
|---|---|---|---|---|
| **Digital Outputs** | 2 | 8 | GPIO / Bus | |
| **Digital Inputs** | 4 | 12 | GPIO / Bus | Active-high |
| **Analog Inputs** | 0 | 4 | ADC1 | 12-bit, DMA |
| **Analog Outputs** | 0 | 2 | DAC | 12-bit DAC |
| **RS-422** | Yes | Yes | USART3 | Crossover option |
| **RS-232** | No | Yes (stub) | UART4 | Reserved, no protocol |
| **Ethernet** | W5500 | W5500 | SPI1 | DHCP/Static |
| **USB** | CDC | CDC | OTG FS | Virtual COM |
| **Status LEDs** | 2 | 1 | GPIO | + per-I/O LEDs on PCB |
| **LCD (optional)** | No | Yes | SPI2 | ST7735 160x80 |
| **HSE Crystal** | 8 MHz | 16 MHz | PLL | 168 MHz core |
| **Board ID** | 0x00 | 0x02 | 4-bit GPIO | Hardwired |

Both boards use the STM32F407VET6 MCU running at 168 MHz. Baby boards use an 8 MHz HSE crystal (PLLM=4), while Mama uses 16 MHz (PLLM=8). All variants include a 4-bit rotary switch for Node ID (0–15). The Mama board has a ribbon cable connector for the optional ST7735 LCD display; the Baby board does not have this connector.

### 1.2 Design Principles

- **Single codebase:** Compile-time defines (`BOARD_BABY`, `BOARD_MAMA`) select hardware-specific code paths. Runtime board ID detection validates the build matches the hardware.
- **Layered architecture:** Hardware abstraction (dio/aio) isolates I/O access from protocol handlers. All communication modules interact through the same I/O API regardless of transport.
- **Configuration over code:** User-settable parameters (device name, I/O labels, MQTT topics, network settings) are stored in flash and editable via the web UI without recompilation.
- **Backward compatibility:** The existing BNS packet protocol (ESC-framed, checksummed) is fully preserved for RS-422, USB, and Ethernet TCP socket communication.
- **Explicit save:** User configuration changes are held in RAM until the user explicitly saves via a dedicated button, preventing accidental changes and minimizing flash wear.

---

## 2. System Architecture

### 2.1 Boot Sequence

The firmware uses a modified startup flow to accommodate the Baby board's different HSE crystal. The CubeMX-generated `main()` immediately calls `newMain()`, which handles clock configuration based on the compile-time board define, then proceeds with peripheral initialization.

1. `HAL_Init()` (called from CubeMX main)
2. `newMain()` called: Board-specific `SystemClock_Config` (Baby: PLLM=4 for 8 MHz, Mama: PLLM=8 for 16 MHz)
3. MX peripheral init: GPIO, DMA, ADC1, UART1/3/4, SPI1/2, DAC, TIM5, USB OTG, RTC
4. `dio_output_enable(false)` — global output disable before any I/O init
5. `board_get_type()` and `node_id_get()` — read hardware straps
6. `dio_init(board)`, `aio_init(board)`, `status_led_init(board)`
7. `dio_all_outputs_off()`, then `dio_output_enable(true)`
8. `config_load()` — load user configuration from flash (or apply defaults if empty/corrupt)
9. `comm_init(node_id)` — initializes packet engine, W5500 Ethernet, RS-422 RX, bridge
10. `app_init()` — loads sequencer apps from flash, validates, sets LED mode
11. LCD init (Mama only, if `lcd_enabled`) — ST7735 display, show boot screen, IP address
12. MQTT client init (if `mqtt_enabled`) — connect to broker, begin state machine
13. Modbus TCP init — open W5500 socket, start listening on port 502
14. Web server init — open W5500 socket, start listening on port 80

### 2.2 Main Loop

The main loop is non-preemptive cooperative multitasking. Each iteration runs all task handlers in sequence:

- `comm_task()` — Process RS-422/USB/Ethernet RX buffers, feed bytes to packet parser, manage W5500 sockets, DHCP polling
- `app_task()` — Execute active sequencer apps, update status LED mode
- `mqtt_process()` — MQTT state machine: DNS, connect, subscribe, publish input changes, handle analog publish timer, reconnect
- `modbus_process()` — Modbus TCP request handling
- `webserver_process()` — Handle pending HTTP requests
- `lcd_update_task()` — Periodic LCD refresh (Mama only, if enabled, throttled to ~250ms)

### 2.3 W5500 Socket Allocation

> ✅ **DECIDED:** 2 sockets for BNS protocol TCP (reduced from 4). Can be further reduced to 1 if socket pressure is encountered.

| Socket | Service | Port | Notes |
|---|---|---|---|
| 0 | BNS Protocol TCP | User-configurable | Packet protocol client 1 |
| 1 | BNS Protocol TCP | User-configurable | Packet protocol client 2 |
| 2 | HTTP Web Server | 80 | Web UI and JSON API |
| 3 | HTTP Web Server | 80 | Second concurrent HTTP client |
| 4 | Modbus TCP | 502 | Single Modbus client |
| 5 | MQTT Client | 1883 (configurable) | Outbound connection to broker |
| 6 | DHCP | 67/68 UDP | Only active during DHCP acquisition |
| 7 | Reserved | — | Future use (DNS, NTP, etc.) |

---

## 3. I/O Subsystem

### 3.1 Digital I/O (dio.c)

The digital I/O layer provides a unified API across both board variants. Baby boards use direct GPIO for 2 outputs and 4 inputs. Mama boards use an 8-bit parallel bus with 3-bit address decoding, active-low output enable, and bidirectional data with direction control. Mama has 8 outputs across 1 bank and 12 inputs across 1.5 banks.

#### 3.1.1 I/O Bus Architecture (Mama)

The bus uses 8 data lines (PD0-PD7), 3 address lines (PE8-PE10), an active-low OE (PE13), a direction pin (PB6, high=write), and a global output enable (PE12, active-low). Output banks are latching: setting the address to an output bank address strobes the current data bus values into that bank's register. The idle address (0x07) must be restored after every write to prevent accidental re-latching.

Address map: 0x00 = Output bank 0 (8 outputs), 0x03–0x04 = Input banks (8 + 4 inputs). Idle address: 0x07.

#### 3.1.2 API

| Function | Description |
|---|---|
| `dio_init(board_type_t)` | Initialize I/O pins/bus for detected board type |
| `dio_output_set(idx, bool)` | Set digital output by index. Baby: direct GPIO. Mama: compute bank/bit, update shadow register, write via bus. |
| `dio_output_get(idx)` | Read output state from shadow register (not re-read from hardware) |
| `dio_input_get(idx)` | Read digital input. Baby: direct GPIO. Mama: select address bank, read data bus, extract bit. |
| `dio_output_enable(bool)` | Global output enable/disable (Mama only, PE12). Safety interlock. |
| `dio_all_outputs_off()` | Clear all outputs to OFF state |
| `dio_get_input_count()` | Returns input count for current board type (Baby=4, Mama=12) |
| `dio_get_output_count()` | Returns output count for current board type (Baby=2, Mama=8) |

### 3.2 Analog I/O (aio.c)

Analog I/O is present on Mama boards only. Analog inputs use the STM32's 12-bit ADC1 with DMA transfer into a RAM buffer. Analog outputs use the on-chip DAC. The aio module mirrors the dio pattern: board type determines counts, and a simple get/set API is provided.

Baby: 0 analog in, 0 analog out. Mama: 4 analog in, 2 analog out.

The BNS analog inputs are general-purpose raw ADC values (not specifically thermistor/temperature channels like the HVAC board). Interpretation and scaling is left to the receiving system or a future user-configurable channel type feature.

---

## 4. Communication Subsystem

### 4.1 Channel Architecture

All communication channels funnel into a unified packet processor. Each physical interface is assigned a channel ID used for routing responses back to the originating interface.

| Channel | ID | Hardware | Notes |
|---|---|---|---|
| RS-422 | `CH_RS422` | USART3 @ 19200 | Interrupt-driven RX, queued TX with DE pin control. Supports normal/crossover wiring. |
| USB CDC | `CH_USB` | OTG FS | Virtual COM port, blocking TX with busy flag |
| Ethernet 0–1 | `CH_ETH_BASE+n` | W5500 SPI1 | 2 concurrent TCP socket connections (sockets 0–1) |
| RS-232 (Mama) | `CH_RS232` | UART4 @ 115200 | Stub only. UART initialized, no protocol assigned. |
| Broadcast | `CH_ALL` | All above | Sends to all active interfaces simultaneously |

### 4.2 Packet Protocol (Existing BNS)

The BNS protocol uses ESC-framed packets with byte stuffing and a simple additive checksum. This protocol is used for RS-422, USB, and raw Ethernet TCP socket communication.

#### 4.2.1 Frame Format

```
[ESC][STX][FromID][Length][ToID][Payload...][Checksum][ESC][ETX]
```

- **ESC** = 0x1B, **STX** = 0x02, **ETX** = 0x03
- **FromID:** Source node ID (0–15)
- **Length:** Payload byte count
- **ToID:** Destination node ID (0–15). Packets not addressed to this node are discarded (future: forwarded via bridge).
- **Checksum:** `(sum of all bytes from first ESC through final ESC) & 0xFF`. If result equals ESC, increment by 1 (VB6 legacy compatibility rule).
- **Byte stuffing:** Any ESC (0x1B) within the data fields is doubled (ESC ESC).

#### 4.2.2 Command Set

| CMD | Name | Description |
|---|---|---|
| 0x01 | `CMD_SETOUTPUTS` | Set digital outputs. Payload: `[maxIdx, startIdx, val0, val1, ...]`. One byte per output (0 or 1). ACK/NAK response. |
| 0x02 | `CMD_GETINPUTS` | Get all inputs. Response: `[numBytes, numDigBytes, digBitmask..., anaLow, anaHigh, ...]` |
| 0x03 | `CMD_APP_STEP_UPDATE` | Write a single app line to a slot. Includes slot, line#, command, port, goto_true, goto_false, param1. |
| 0x04 | `CMD_BEGIN_END_APP` | Start (action=1) or stop (action=0) an app in a given slot. |
| 0x05 | `CMD_APP_MESSAGE` | Send an inter-app message to a specific slot. |
| 0x06 | `CMD_APP_STATUS` | Query app status: returns outgoing_msg, current_line, running state. |
| 0x07 | `CMD_GETOUTPUTS` | Get all digital output states as bitmask. |
| 0x08 | `CMD_EXTENDED_FUNCS` | Reserved for future extended commands. |
| 0x09 | `CMD_CONFIG_LAYOUT` | Reserved for board layout/config queries. |
| 0x55 | `CMD_ACK` | Acknowledgment. |
| 0xAA | `CMD_NAK` | Negative acknowledgment. |

### 4.3 Communication Bridge

The bridge module enables protocol-transparent forwarding between interfaces. Currently supports RS-422-to-Ethernet and RS-422-to-USB forwarding modes. Bridge modes are configured per-interface and can be set at runtime. Future: full mesh bridging between all interfaces.

### 4.4 RS-232 (Mama Only)

> ✅ **DECIDED:** RS-232 (UART4) is initialized at 115200 baud but has no protocol assigned. The UART peripheral is configured and ready, but no `comm_task` processing is wired up. This serves as a stub for future peripheral control use cases. Implementation may remain a permanent stub.

---

## 5. Ethernet & Network Services

### 5.1 W5500 Ethernet Controller

All BNS boards use the WIZnet W5500 hardwired TCP/IP controller connected via SPI1. The W5500 provides 8 independent hardware sockets allocated as described in Section 2.3. Network configuration supports both static IP and DHCP, selectable via the web UI settings page.

### 5.2 Modbus TCP Server

Ported from the HVAC RemCon board's implementation, adapted for the W5500 socket API. Listens on port 502 (socket 4). The Modbus Unit ID corresponds to the board's Node ID (rotary switch).

#### 5.2.1 Register Map

| FC | Address | Type | Description |
|---|---|---|---|
| 0x01 | 0x0000+ | Read Coils | Digital output states. Baby: 2 coils, Mama: 8 coils. |
| 0x02 | 0x0000+ | Read Discrete Inputs | Digital input states. Baby: 4, Mama: 12. |
| 0x04 | 0x0000+ | Read Input Registers | Analog input raw values (16-bit). Mama: 4 registers. |
| 0x05 | 0x0000+ | Write Single Coil | Set one digital output. 0xFF00=ON, 0x0000=OFF. |
| 0x06 | 0x0000+ | Write Single Register | Set one analog output value (16-bit). Mama: 2 registers. |
| 0x0F | 0x0000+ | Write Multiple Coils | Set multiple digital outputs at once. |
| 0x10 | 0x0000+ | Write Multiple Registers | Set multiple analog output values. |

Quantity clamping: If a request exceeds available I/O count, the response is clamped to available points rather than returning an exception (matching HVAC board behavior).

### 5.3 Web Server & UI

The web server provides a browser-based interface for monitoring, control, and configuration. It serves embedded HTML/CSS/JS from flash memory and provides JSON API endpoints for dynamic data. No authentication is required for initial release; the board is assumed to be on a trusted network.

> ✅ **DECIDED:** No web authentication for initial release. May be added as an optional feature in a future revision.

#### 5.3.1 Pages

- **Dashboard (`/`)** — Live view of all digital inputs, digital outputs (clickable toggles), and analog I/O values. I/O elements display user-configured names. Auto-refreshes via JSON polling at 500ms.
- **Settings (`/settings`)** — Configuration page for all user-settable parameters (see Section 8). Changes are held in RAM until the user presses a dedicated "Save to Flash" button.
- **Apps (`/apps`)** — View and manage sequencer apps: status display, start/stop controls.

#### 5.3.2 JSON API Endpoints

| Endpoint | Method | Description |
|---|---|---|
| `/status.json` | GET | All I/O states: digital in/out values, analog in/out values, app statuses, MQTT connected status |
| `/toggle?num=N` | GET | Toggle digital output N (by index) |
| `/config.json` | GET | Read entire user configuration (device name, I/O names, MQTT settings, network settings) |
| `/config.json` | POST | Write user configuration to RAM (NOT flash). Validates I/O name uniqueness before accepting. |
| `/config/save` | POST | Commit RAM configuration to flash. This is the only action that writes to the config flash sector. |
| `/config/revert` | POST | Discard RAM changes and reload configuration from flash. |
| `/app/N/start` | POST | Start app in slot N (0–3) |
| `/app/N/stop` | POST | Stop app in slot N (0–3) |
| `/info.json` | GET | Board type, firmware version, node ID, uptime, IP address, MQTT connection state |

---

## 6. MQTT Integration

MQTT provides pub/sub communication for integration with home automation platforms (Home Assistant, Node-RED, OpenHAB, etc.), SCADA systems, and inter-device coordination. The implementation builds on the HVAC RemCon board's MQTT client, extended with full user configurability.

### 6.1 Topic Structure

The topic hierarchy is constructed from user-configurable components:

- **Root Topic:** Set by the user in the web UI settings page. Example: `/device/hotel`
- **Node ID Suffix (optional):** When enabled, the Node ID (from the rotary switch) is appended with an underscore separator. Example: `/device/hotel_2` for Node ID 2. The suffix updates dynamically if the rotary switch is changed.
- **I/O Name:** Each I/O point has a user-configurable name. This name becomes the final topic segment and is used both in MQTT topics and in the web UI dashboard.

**Full topic example:** Root = `/device/hotel`, Node ID append enabled (ID=2), digital input 3 named "yellow" → published on `/device/hotel_2/yellow`

#### 6.1.1 Default Topic Names

If the user has not customized I/O names, defaults are used:

- Digital inputs: `din0`, `din1`, `din2`, ... `dinN`
- Digital outputs: `dout0`, `dout1`, `dout2`, ... `doutN`
- Analog inputs: `ain0`, `ain1`, `ain2`, ... `ainN`
- Analog outputs: `aout0`, `aout1`, `aout2`, ... `aoutN`

#### 6.1.2 Topic Uniqueness

> ✅ **DECIDED:** I/O names must be unique across ALL I/O types (digital and analog, inputs and outputs). The web UI and `/config.json` POST endpoint validate uniqueness and reject duplicate names with an error message indicating which names conflict. Validation is case-sensitive.

### 6.2 Publish Behavior

#### 6.2.1 Digital I/O Publishing

Digital I/O state changes are detected via XOR comparison of the full input/output bank against the last published state, checked every 50ms (matching the HVAC board polling interval). On change, only the changed points are published.

#### 6.2.2 Customizable State Values

Each digital I/O point has user-configurable strings for the ON and OFF states:

- Default ON value: `"1"`
- Default OFF value: `"0"`
- User can set any string, e.g., ON=`"Open"`, OFF=`"Closed"` or ON=`"ACTIVE"`, OFF=`"IDLE"`
- A per-I/O option (`mqtt_value_is_string`) selects whether the value is sent as a quoted JSON string or as a raw/numeric payload.

#### 6.2.3 Analog Input Publishing

> ✅ **DECIDED:** Analog inputs are published on a user-configurable interval (default: 60 seconds). Additionally, any message received on a "request" topic triggers an immediate publish of the current value for that analog input.

Analog values are published as numeric strings (e.g., `"2048"`) to the standard analog input topic. The request topic for triggering an on-demand read uses the same topic path. When the board receives any message on an analog input's topic (which it subscribes to in addition to output topics), it immediately publishes the current raw ADC value in response. This provides a simple request/response mechanism: a remote device publishes anything to `/device/hotel_2/ain0` and the board responds by publishing the current value back to that same topic.

**Implementation detail:** The board subscribes to analog input topics as well as digital output topics. For analog input topics, incoming messages trigger an immediate publish rather than controlling an output. For digital output topics, incoming messages control the output state.

> ✅ **DECIDED:** MQTT QoS level 1 (at-least-once delivery with acknowledgment) for all published messages. This adds reliability over QoS 0 at the cost of requiring the W5500 MQTT socket to handle PUBACK responses.

> ✅ **DECIDED:** MQTT retain flag is enabled for all published messages. New subscribers immediately receive the last-known state of all I/O points upon subscribing.

### 6.3 Subscribe Behavior

The client subscribes to:

- **Digital output topics** — Incoming payload is compared against the configured ON/OFF values for that output. If matched, the output is set accordingly. Unrecognized payloads are ignored.
- **Analog output topics (Mama only)** — Incoming payload is parsed as a numeric value (0–4095) and applied to the DAC output.
- **Analog input topics** — Any incoming message triggers an immediate publish of the current analog value (request/response pattern).

### 6.4 Connection Management

The MQTT state machine follows the pattern established in the HVAC board:

- IDLE → DNS_LOOKUP → CONNECTING → SUBSCRIBING → CONNECTED
- On disconnect or error: RECONNECT_WAIT (configurable delay, default 5s) → IDLE
- Broker hostname, port, client ID prefix, optional username/password are user-configurable.
- Client ID is auto-generated: configured prefix + board IP or MAC address.
- Keep-alive interval: configurable (default 60s).

### 6.5 MQTT in Sequencer Apps (New Commands)

Two new app commands extend the sequencer engine to interact with MQTT:

| Command | Description |
|---|---|
| `APP_CMD_MQTT_PUBLISH` | Publish a predefined message value to a configurable MQTT topic. Param1 holds a topic/message index into a lookup table stored in the app config. |
| `APP_CMD_MQTT_WAIT` | Block/branch based on current MQTT topic payload matching a specified value. Similar to `WAIT_INPUT_TRUE` but for MQTT state. Supports `goto_true`/`goto_false` for conditional branching. |

**Implementation note:** MQTT topics/payloads referenced by app commands use index-based lookup into a table to avoid storing variable-length strings in the fixed-format `app_line_t` structure. The lookup table is stored alongside the app data in flash.

---

## 7. Sequencer Engine (Apps)

The sequencer engine runs up to 4 concurrent programs ("apps") stored in on-chip flash (Sector 11, 128KB). Each app consists of a fixed-size array of instruction lines executed sequentially with support for conditional branching, delays, inter-app messaging, and I/O control.

### 7.1 App Structure

| Field | Type | Description |
|---|---|---|
| `app_version` | `char[10]` | User-defined version string (printable ASCII 0x20–0x7E) |
| `num_lines` | `uint8_t` | Number of valid lines in this app |
| `lines[MAX_LINES]` | `app_line_t[]` | Array of instruction lines |
| `running` | `bool` | Whether this app is currently executing |
| `current_line` | `uint8_t` | Current program counter |
| `in_delay` | `bool` | Currently in a DELAY instruction |
| `delay_end_time` | `uint32_t` | `HAL_GetTick()` target for delay completion |
| `incoming_msg` | `uint8_t` | Last received inter-app message |
| `outgoing_msg` | `uint8_t` | Last sent inter-app message (readable by host via `CMD_APP_STATUS`) |
| `has_msg` | `bool` | Flag: unread incoming message available |

### 7.2 Instruction Set

| Command | Behavior |
|---|---|
| `ACTIVATE_OUTPUT` | Set `output[port]` = ON. Advance to next line. |
| `DEACTIVATE_OUTPUT` | Set `output[port]` = OFF. Advance to next line. |
| `TEST_INPUT` | If `input[port]` is ON, goto `goto_true`; else goto `goto_false`. |
| `WAIT_INPUT_TRUE` | Block until `input[port]` is ON, then advance. |
| `WAIT_INPUT_FALSE` | Block until `input[port]` is OFF, then advance. |
| `DELAY` | Wait `param1` milliseconds (non-blocking via `HAL_GetTick` comparison). |
| `GOTO` | Unconditional jump to `goto_true`. |
| `WAIT_FOR_MSG` | Block until `incoming_msg` matches `param1`, then advance and clear msg flag. |
| `SEND_MSG` | Set `outgoing_msg` = `param1`. Advance. Host reads via `CMD_APP_STATUS`. |
| `TEST_FOR_MSG` | If `incoming_msg` matches `param1`, goto `goto_true` (and clear msg); else `goto_false`. |
| `ALL_OUTPUTS_OFF` | Turn off all digital outputs via `dio_all_outputs_off()`. Advance. |
| `MQTT_PUBLISH` *(new)* | Publish a predefined MQTT message. `param1` = message table index. Advance. |
| `MQTT_WAIT` *(new)* | Block/branch based on MQTT topic payload. `param1` = topic/value table index. Supports `goto_true`/`goto_false`. |

### 7.3 Flash Storage

Apps are stored as a raw memory image of the `apps[]` array in flash Sector 11 (0x080E0000). On boot, the entire sector is `memcpy`'d into RAM and validated. Invalid apps (bad version chars, out-of-range ports/jumps, unknown commands) are zeroed out. Flash writes use single-byte programming after a full sector erase.

---

## 8. User Configuration System

All user-configurable parameters are stored in a dedicated flash sector (Sector 10, separate from app storage) as a serialized configuration structure. The configuration system uses an explicit-save model to protect against accidental changes and minimize flash wear.

### 8.1 Save Model

> ✅ **DECIDED:** Configuration changes made via the web UI are held in RAM only. The user must press a dedicated "Save to Flash" button on the settings page to commit changes. A "Revert" button discards RAM changes and reloads from flash. The web UI clearly indicates when unsaved changes exist (e.g., a banner or color change). This prevents accidental misconfiguration and avoids unnecessary flash wear from rapid setting changes during exploration/testing.

### 8.2 Configuration Parameters

#### 8.2.1 Device Identity

| Parameter | Type | Default | Description |
|---|---|---|---|
| `device_name` | `char[32]` | `"BNS-Baby"` / `"BNS-Mama"` | User-friendly device name (shown on LCD and web UI) |
| `lcd_enabled` | `bool` | `false` | Mama only. Whether LCD is connected/active. |

#### 8.2.2 Network Settings

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ip_mode` | `uint8_t` | 0 (Static) | 0=Static, 1=DHCP |
| `static_ip` | `uint8_t[4]` | 192.168.1.100 | Static IP address |
| `subnet` | `uint8_t[4]` | 255.255.255.0 | Subnet mask |
| `gateway` | `uint8_t[4]` | 192.168.1.1 | Default gateway |
| `dns` | `uint8_t[4]` | 8.8.8.8 | DNS server (for MQTT hostname resolution) |

#### 8.2.3 MQTT Settings

| Parameter | Type | Default | Description |
|---|---|---|---|
| `mqtt_enabled` | `bool` | `false` | Enable/disable MQTT client |
| `mqtt_broker_host` | `char[64]` | `""` | Broker hostname or IP |
| `mqtt_broker_port` | `uint16_t` | 1883 | Broker port |
| `mqtt_username` | `char[32]` | `""` | Optional auth username |
| `mqtt_password` | `char[32]` | `""` | Optional auth password |
| `mqtt_root_topic` | `char[64]` | `"/bns/dev"` | Root topic prefix |
| `mqtt_append_node_id` | `bool` | `true` | Append `_N` (Node ID) to root topic |
| `mqtt_client_id_prefix` | `char[16]` | `"bns_"` | Client ID prefix (IP/MAC appended auto) |
| `mqtt_keepalive_sec` | `uint16_t` | 60 | Keep-alive interval (seconds) |
| `mqtt_reconnect_ms` | `uint32_t` | 5000 | Reconnect delay after disconnect (ms) |
| `mqtt_analog_interval_s` | `uint16_t` | 60 | Analog input publish interval (seconds). 0 = only on-demand. |

#### 8.2.4 I/O Names and MQTT State Values

Each I/O point has a configurable name and (for digital I/O) configurable ON/OFF state strings. Stored as a per-I/O config array sized to the maximum I/O count for the Mama board.

| Field | Type | Default | Description |
|---|---|---|---|
| `name` | `char[16]` | `"din0"`/`"dout0"`/etc. | I/O name. Must be unique across all I/O. Used in web UI and MQTT topic. |
| `mqtt_on_value` | `char[16]` | `"1"` | Payload string published/matched when ON (digital only) |
| `mqtt_off_value` | `char[16]` | `"0"` | Payload string published/matched when OFF (digital only) |
| `mqtt_value_is_string` | `bool` | `false` | `false` = raw payload (`1`), `true` = JSON string (`"1"`) |

### 8.3 Flash Storage Layout

Configuration is stored in flash Sector 10. A magic number and version field at the start allow detection of uninitialized or incompatible configurations, falling back to defaults. A CRC32 protects against corruption.

Structure: `[MAGIC_WORD 0xBN50CF61 (4B)] [CONFIG_VERSION (2B)] [config_data (...)] [CRC32 (4B)]`

On boot, if the magic word is missing, version is unrecognized, or CRC fails, the entire config is populated with defaults and **NOT** written to flash (the user can choose to save the defaults or configure first).

#### 8.3.1 Version Migration Strategy

> ✅ **DECIDED:** For now, no automatic migration between config versions. If `CONFIG_VERSION` changes between firmware updates, the config sector is treated as invalid and defaults are loaded. Users will need to reconfigure via the web UI. A future revision may add field-by-field migration for minor version bumps.

---

## 9. LCD Display (Mama Only)

The Mama board has a ribbon cable connector for the ST7735 160x80 LCD, connected via SPI2. The LCD is optional and controlled by the `lcd_enabled` configuration flag. If disabled (or on Baby boards, which lack the connector), all display calls are no-ops.

> ❌ **REMOVED:** Baby boards do NOT have the LCD ribbon cable connector. LCD code must be compiled out or no-op'd for Baby builds.

### 9.1 Display Layout

Since BNS boards have dedicated per-I/O LEDs on the PCB, the LCD does not need to show individual I/O states. The display focuses on system-level information:

**Status Bar (top row, 12px height):**
- IP address (left-aligned, small font)
- Node ID (right-aligned, small font)

**Main Area:**
- Device name (user-configurable, centered, large font)
- App status summary: 4 compact rows showing each slot's state (empty / stopped / running + current line#)
- MQTT status: "MQTT: Connected" or "MQTT: Disconnected" or "MQTT: Off"
- Network status indicator (DHCP pending / connected / link down)

### 9.2 Update Strategy

LCD updates are performed in the main loop at a throttled rate (~250ms minimum between full redraws) to avoid excessive CPU time. The LCD is on SPI2 while the W5500 is on SPI1 (separate buses), but CPU time is the constraint in the cooperative loop. The ST7735 driver from the HVAC board (ST7735Canvas) is reused with the same font set (Dialog 10pt narrow for status bar, Dialog 24pt for large text).

---

## 10. Status LED System

The status LED system provides visual feedback for system state and communication activity. Baby boards have 2 LEDs (active-low on PA2/PA3). Mama has 1 LED (active-high on PA2). The driver auto-adapts based on board type.

### 10.1 LED Modes

| Mode | Pattern | Meaning |
|---|---|---|
| `BOOTING` | OFF | System is initializing |
| `NOAPP` | 250ms ON / 3750ms OFF | No apps loaded in flash |
| `STOPPED` | 250ms ON / 1250ms OFF | App(s) loaded but none running |
| `RUNNING` | 250ms ON / 250ms OFF | At least one app is running (2 Hz blink) |

**Data activity overlay:** On 2-LED boards (Baby), LED2 pulses for 50ms on serial/Ethernet traffic. On single-LED boards (Mama), a smart overlay briefly inverts the main pattern (10ms OFF, 30ms ON, 40ms OFF) to indicate activity without disrupting the status indication.

---

## 11. Implementation Roadmap

### Phase 1: Core Infrastructure

- Implement user configuration storage system: flash sector management, config struct with magic/version/CRC, default population, load/save API
- Port HVAC webserver to W5500 socket-based HTTP (sockets 2–3)
- Build settings page with explicit Save/Revert buttons and unsaved-changes indicator
- Wire I/O names from config into web UI dashboard
- Implement LCD display module with conditional enable (Mama only)
- Remove/stub Papa board code paths

### Phase 2: Modbus TCP

- Port HVAC Modbus TCP to W5500 socket API (socket 4)
- Add FC 0x06 (Write Single Register) and FC 0x10 (Write Multiple Registers) for analog outputs
- Test with Modbus poll tools

### Phase 3: MQTT Integration

- Implement MQTT client over W5500 TCP socket (socket 5) with QoS 1 and retain
- Implement configurable topic hierarchy: root + optional Node ID suffix + I/O name
- Implement customizable ON/OFF state values with string/number type option
- Implement analog input periodic publishing with configurable interval
- Implement analog input on-demand publishing via subscription/request pattern
- Add I/O name uniqueness validation
- Add MQTT settings to web UI settings page
- Test with Home Assistant / Node-RED / MQTT Explorer

### Phase 4: Sequencer Extensions

- Add `MQTT_PUBLISH` and `MQTT_WAIT` app commands to sequencer engine
- Implement MQTT topic/message lookup table for app commands
- Add app management controls to web UI (`/apps` page)

### Phase 5: Polish & Hardening

- Watchdog timer integration
- Error logging and diagnostics
- Full bridge routing between all interfaces
- OTA firmware update (stretch goal)
- Modbus RTU over RS-422 (stretch goal)
- Web authentication option (stretch goal)

---

## 12. Decision Log

All open questions from v1.0 have been resolved:

| Question | Decision | Section |
|---|---|---|
| W5500 socket allocation | 2 for BNS protocol, 2 HTTP, 1 Modbus, 1 MQTT, 1 DHCP, 1 reserved | §2.3 |
| MQTT QoS level | QoS 1 (at-least-once) | §6.2 |
| MQTT retain flag | Enabled for all published messages | §6.2 |
| Analog input publishing | Configurable interval (default 60s) + on-demand via topic subscription | §6.2.3 |
| Config versioning | Reset to defaults on version mismatch. No auto-migration for now. | §8.3.1 |
| Papa board | Cancelled. Removed from spec. `BOARD_TYPE_PAPA` retained as reserved enum. | §1 |
| Flash wear / config save | Explicit save button only. No automatic writes. | §8.1 |
| RS-232 usage | Stub only. UART initialized, no protocol. Present on Mama. | §4.4 |
| MQTT topic uniqueness | Required. Enforced by web UI with validation error on duplicates. | §6.1.2 |
| Web authentication | None for initial release. Stretch goal for future. | §5.3 |
| LCD on Baby boards | Not available. Baby boards lack the ribbon connector. | §9 |
