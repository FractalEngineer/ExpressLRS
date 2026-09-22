# ELRS MAVLink Lua parameter bridge handoff

Updated: 2026-09-22. Development work; PR [#3779](https://github.com/ExpressLRS/ExpressLRS/pull/3779) is open against `master` as a draft. All of pkendall64's first-pass review comments are addressed and the branch is pushed to the fork. The ladder-window/download path remains abandoned.

**The vehicle-discovered parameter index also failed on hardware** and must not be treated as the agreed direction. On the Lua side the on-radio build reports `not enough memory for buffer allocation` from around 700 names and sometimes freezes the radio, and no combination of bounded writes, cache preloading, or libraries fixed it. The attempt is archived on the MAV-LUA `self-building-index` branch. The bounded `PARAM_REQUEST_LIST` list session built to serve it is implemented here (see "List session") and passes its native tests, but nothing consumes it yet, so it should be treated as an unproven prerequisite rather than a step towards a working feature. How parameter names are obtained is once again an open question.

## Current state

The TX-only bridge, four-request window, paced one-shot `SYS_STATUS` requester, CRSF `0xAC` readiness conversion and the CRSF `0xAA` handset envelope are implemented on `feature/mavlink-lua-parameters`. Native tests and the RadioMaster Zorro ESP32 build pass.

Interception point: the legacy `0xAA` envelope is claimed in `TXModuleEndpoint::handleRaw`, not in `CRSFHandset::ProcessPacket`. The TX module must own the frame type exclusively, because its first payload bytes are a chunk marker and MAVLink length rather than CRSF addresses. Review explicitly rejected intercepting in the handset before routing.

| Item | Value |
| --- | --- |
| Checkout | C:/Users/titan/Desktop/Github_Projects/ExpressLRS |
| Active branch | feature/mavlink-lua-parameters |
| Base | a60b68af521d546fd395674d66e83291d035ae61 |
| Base commit subject | Merge pull request #3711 from ExpressLRS/gps-autoconfig-ubx |
| origin | https://github.com/ExpressLRS/ExpressLRS |
| myfork | https://github.com/FractalEngineer/ExpressLRS.git |
| Lua checkout | C:/Users/titan/Desktop/Github_Projects/Mine/MAV-LUA |

The branch was created from this checkout's origin/master. It is not pinned to the ELRS 4.1.0 tag, although the initial bridge prototype was developed against that tag. The implementation is in modified and untracked files: sharing the branch name alone does not transfer it.

The user wants a working hardware checkpoint before committing. Preserve this working tree and continue locally until they confirm it. Do not automatically commit, push, open a PR or flash a module.

## Purpose and hardware

MAV-LUA provides Navigation, Messages and an opt-in autopilot Parameters page. Stock ELRS telemetry conversion does not expose PARAM_VALUE replies to radio Lua. CRSF device parameters configure the TX module; they are separate from ArduPilot parameters.

The user's radio is a TBS Alpha on EdgeTX 2.11. Its external-bay full-duplex serial connection is wired to a RadioMaster Zorro internal ELRS module, reported to be running ELRS 4.1. The first build target is that Zorro module. Preserve the user's actual wiring, binding and radio-domain settings when preparing an upgrade; the successful compile does not establish that the build matches every custom setting.

This is a TX-only change using the existing MAVLink OTA path. No RX or EdgeTX firmware change is included. Normal sensor and status-message conversion are retained.

An external ELRS module using the bay's S.Port-labelled pin for half-duplex CRSF is expected to use the same bridge, but that physical configuration has not been tested. Actual FrSky S.Port is a different protocol and requires a separate Lua/MAVLite transport; this change does not implement it.

## Files and integration

Paths below are relative to this ExpressLRS repository.

| File | Role |
| --- | --- |
| src/lib/MavLuaBridge/MavLuaBridge.h | Portable parameter session plus readiness target/pacing state. |
| src/lib/MavLuaBridge/MavLuaTransport.h | Handset enqueue, main-loop poll and downlink interfaces. |
| src/lib/MavLuaBridge/MavLuaTransport.cpp | TX-only FIFO, existing uplink integration and CRSF reply framing. |
| src/lib/tx-crsf/TXModuleEndpoint.cpp | Claims the handset 0xAA frame type in handleRaw, so the router never reads its chunk/length bytes as extended addresses. |
| src/src/tx_main.cpp | Calls mavLuaPoll() before processing completed downlink data. |
| src/lib/MAVLink/MAVLink.cpp | Hooks HEARTBEAT/SYS_STATUS into readiness tracking, validated unsigned HEARTBEAT/PARAM_VALUE into the parameter bridge, and converts `SYS_STATUS` masks to CRSF `0xAC`. |
| src/include/crsf_protocol.h | Names standard CRSF frame type `0xAC` for MAVLink system status. |
| src/test/test_mavlua/test_mavlua.cpp | Native Unity protocol test with independently calculated request CRCs. |

The library has its own directory so native tests can exercise the portable header without pulling in embedded MAVLink converter dependencies. Keep firmware source here. The duplicate bridge sources, generated patch and patch generator were removed from MAV-LUA; do not recreate a second source of truth there.

## Data path and wire contract

Uplink: Lua crossfireTelemetryPush -> handset CRSF validation -> bounded payload queue -> mavLuaPoll on TX main loop -> existing uartInputBuffer / MAVLink OTA uplink -> receiver / autopilot.

Downlink: autopilot -> existing ELRS MAVLink parser -> bridge subscription/target/index filtering -> CRSF 0xAA to RADIO_TRANSMITTER -> MAV-LUA's sole crossfireTelemetryPop consumer.

Readiness path: fresh ArduPilot heartbeat -> at most 1 Hz one-shot `MAV_CMD_REQUEST_MESSAGE(SYS_STATUS)` while status is absent/older than two seconds -> normal ELRS converter -> standard CRSF `0xAC` containing big-endian present/enabled/health masks -> the same sole Lua queue consumer. Natural `SYS_STATUS` streaming suppresses requests. This path is independent of parameter subscription.

- CRSF payload is [0, mavlink_packet_length, complete_mavlink_packet...]. Only a single chunk is supported. There is no destination/origin prefix in this payload. The restricted subscription behavior below is this implementation's convention.
- Uplink uses unsigned MAVLink 1, source system 254 and component 190. Only PING (4, CRC extra 237), PARAM_REQUEST_READ (20, extra 214), and PARAM_SET (23, extra 168) are accepted, with exact payload lengths and valid CRCs. A strict `MAV_CMD_REQUEST_MESSAGE(AUTOPILOT_VERSION)` (76) is also accepted for firmware identity. `PARAM_REQUEST_LIST` (21) is also accepted here, but only inside a bounded list session (see "List session"). That session was written to feed a vehicle-discovered index which then failed on radio, so it is currently exercised only by native tests.
- A PING with all 14 payload bytes zero opens/refreshes a local ten-second subscription. It is not forwarded to the aircraft. Lua sends nothing before explicit Load; a local keepalive runs approximately every three seconds while the loaded Parameters page is visible.
- The first subscribed ArduPilot HEARTBEAT (autopilot == 3, component 1, nonzero system ID) selects the vehicle. Other system IDs are ignored during that session. Requests must target that system and component 1.
- The transport operates only in TX_MAVLINK_MODE with connectionState == connected. Otherwise the poll resets the session and clears the handset queue.
- A separate readiness tracker accepts a fresh ArduPilot heartbeat without a Lua PING. It queues `MAV_CMD_REQUEST_MESSAGE` for `SYS_STATUS` from GCS 254/190 only when the uplink FIFO has capacity. It retries no faster than once per second, stops while status is fresh and resets on link loss.
- Read requests have a 20 ms bridge floor; the Lua client paces them at 40 ms or slower. Read indices are restricted to 0 through 8191, with four outstanding indices and one forwarded reply per index. Duplicate retries reuse their slot; abandoned slots expire after five seconds. A 100 ms gap is required before SET and after SET. SET clears old read subscriptions. An index is reserved only when the uplink FIFO has room.
- PARAM_SET requires a discovered, disarmed heartbeat no older than three seconds. This is a status gate, not an atomic guarantee against the aircraft arming after that heartbeat.
- Downlink packets are already validated by ELRS's parser. Only unsigned HEARTBEAT/PARAM_VALUE replies are considered. The Lua decoder accepts MAVLink 1 and 2, including v2 trailing-zero truncation. The adapter excludes signed packets.
- SET acknowledgements are not independently subscribed to. Lua follows the write with an indexed read and verifies the returned name/type/value/target before reporting success.
- Handset input uses a locked 128-byte FIFO. Bridge state and aircraft writes run on the TX main loop. A native static assertion keeps retained bridge state at or below 80 bytes. Replies are limited to 58 MAVLink bytes within a 64-byte CRSF frame.

This is a restricted parameter bridge, not arbitrary MAVLink forwarding. Multi-chunk handset framing, `PARAM_EXT`, signed traffic and PX4 bytewise parameter encoding are unsupported. Streamed `PARAM_REQUEST_LIST`/`PARAM_VALUE` traffic is not forwarded yet; the bounded list session is the next change. Lua additionally rejects unsupported numeric types/values; the bridge does not independently implement all editor numeric validation.

## Lua counterpart and failure semantics

In the separate MAV-LUA repository:

| File | Role |
| --- | --- |
| src/SCRIPTS/TELEMETRY/MAV.lua | Sole custom-frame consumer and lazy Parameters loader. |
| src/SCRIPTS/MAV/wire.lua | MAVLink framing/CRC and parameter decoding. |
| src/SCRIPTS/MAV/params.lua | Opt-in session, exact-name reads, edit/review/save and verification. |
| src/SCRIPTS/MAV/pdb.lua | Fixed-record index/category pager: seek, then read at most eight records. |
| src/SCRIPTS/MAV/pview.lua, pinput.lua | Bounded drawing and controls for the Parameters page. |
| tests/test_params.lua | Independent protocol vectors, bounded paging, failure and integrated UI tests. |
| AGENTS.md | Detailed Lua implementation and release constraints. |

The v0.1.3 release browser reads a fixed-record name file locally: fixed 22-byte category records and 16-byte names, a seek plus at most 128 bytes per name page, and an exact-name read only on ENTER. That pager is reusable, and its packaged name source is once again the shipped one, because the vehicle-discovered index meant to replace it failed on radio. The earlier indexed-window download (four outstanding reads, 512-byte block commits, adaptive 40-200 ms spacing, 0.6-2.4 s adaptive timeout, four attempts per index, and the now-removed `fetch.lua`/`store.lua`) is abandoned.

crossfireTelemetryPush success only means the radio accepted the frame; it does not acknowledge TX forwarding or autopilot application. FIFO pressure, rate limiting and RF loss can still drop traffic. A write is never automatically repeated after radio acceptance. An unconfirmed save must be inspected by reopening the parameter.

Historical failed candidates, kept so they are not retried blindly: the named `dist/params-dev5/` candidate failed with `not enough memory for buffer allocation`; a later candidate cold-opened and reached 6-9 parameters/second but hard-froze after 30-69 records, with the UI and power-off unresponsive while telemetry-lost audio still worked. Do not ask the user to retest either package. Those failures were in the Lua-side list retention and per-record SD write strategy rather than in the transport, which is why the replacement builds a fixed-record index in bounded runs.

Exactly these two package types are intended for future tags. Modern compiled Lua is only an internal host-test artifact. The historical MAV-LUA v0.1.1 release remains unchanged and has two pages with Enter page switching; it cannot exercise this bridge.

Update Lua and the TX bridge together: the previous TX build does not request missing `SYS_STATUS`. The failed hardware run remained `RDY?` while disarmed and changed to the old `ARMD` label when armed, proving `0x5001` but not readiness transport. The current Lua spells that state `ARMED`. The candidate uses PAGE for Navigation -> Messages -> Parameters; Enter selects. The Parameters prompt opens without patched TX firmware, but Load times out without bridge replies. If Enter still changes pages, old Lua is running: clear MAV's `.luac` caches, copy the complete source package and restart. If the telemetry menu owns PAGE, launch MAV from Tools.

## Reproduce the build and test

Run these PowerShell commands from the native checkout:

    Set-Location C:/Users/titan/Desktop/Github_Projects/ExpressLRS/src
    pio test -e native -f test_mavlua
    $env:ELRS_UNIFIED_CONFIG='radiomaster.tx_2400.zorro'
    pio run -e Unified_ESP32_2400_TX_via_UART

If pio is absent from PATH, replace it with:

    & C:/Users/titan/.platformio/penv/Scripts/pio.exe

Use build-only commands. Adding an upload target flashes hardware and is a separate action.

The environment variable selects Zorro hardware noninteractively without changing tracked target settings. The existing nested src/hardware checkout had many deleted JSON files. Only targets.json and TX/Radiomaster Zorro.json were restored to finish this build. Other pre-existing deletions were left alone; do not blindly reset or restore that checkout. It is ignored by the parent repository, so inspect its own Git status when diagnosing missing definitions.

Build output: src/.pio/build/Unified_ESP32_2400_TX_via_UART/firmware.bin, alongside firmware.elf and supporting images. PlatformIO completed image generation and hardware configuration.

| Recorded check | Result |
| --- | --- |
| Native Unity: protocol, pipeline, write gate, readiness pacing | Four tests pass, including readiness target selection, missing/fresh-status pacing and clock wrap. |
| Complete native-checkout Zorro ESP32 build | Passed after the requester: 71,112 bytes RAM and 1,614,957 bytes flash reported (21.7% / 82.1%). These are total firmware sizes, not incremental bridge cost. |
| Lua source / host bytecode / legacy VM and packaged-entry checks | Passed in MAV-LUA, including stale cache selection, PAGE/Enter behavior, opt-in loading and fast-callback request pacing. |
| Hardware/RF, modified Alpha wiring, half-duplex module | Not yet validated. |

Logs are currently in the Lua checkout at .build/elrs-pipeline-test.log and .build/elrs-pipeline-build.log. These are ignored local artifacts, not committed evidence. Unity tests portable bridge logic, not the full handset/FIFO/RF path. The complete build validates compilation and linking, not RF behavior.

## Pipeline validation and measurements

The Lua loading screen displays observed parameters per second. Idle visible loading callbacks consume up to two parameter frames; key-event and Navigation callbacks retain the one-frame custom-packet budget. No persistent cross-session cache or PARAM_REQUEST_LIST was added at the time of this measurement; the bounded list session described under "List session" supersedes that ladder-window approach.

The Lua pipeline test covers delayed/reordered/duplicate replies, loss, congestion, pause/resume, PAGE transitions, aligned block ordering and short-write recovery. Real-core stress includes MAVLink 2 and dense status messages; the current maximum is 8,222 instructions against a 10,000 limit.

On a controlled simulation with a 100 ms round trip and 25 replies/second capacity, 160 records take 8.00 seconds. On Alpha the prior per-record writer reached 6-9 parameters/second but froze after 30-69 records. The aligned block writer has not run on radio.

## List session

The bounded list session is implemented on the `feature/mavlink-lua-parameter-list` branch, which carries it separately from the reviewed PR so the PR stays free of the failed work. It accepts `PARAM_REQUEST_LIST`, forwards streamed `PARAM_VALUE` without a per-index reservation while open, and is hard-capped by time and packet count, lease-gated, and mutually exclusive with normal reads. Native tests cover the session and its packet ceiling, and all seven `test_mavlua` cases pass.

Nothing consumes it. It was built to serve the vehicle-discovered index, which failed on radio, so treat it as an unproven prerequisite: it does what its tests say, but no working feature depends on it.

## Next work

**How parameter names are obtained is an open question.** Both previous answers failed on hardware. The ladder-window download froze the radio after 30-69 records. The vehicle-discovered index got further, then reported `not enough memory for buffer allocation` from around 700 names and sometimes froze the radio, and it did so after per-record writes, cache preloading, removal of the `table` library dependency and removal of per-record string padding had all been tried. Do not restart either path without new evidence that the whole operation fits the radio heap.

A useful next step is to stop treating this as a Lua problem and measure the actual ceiling directly: what the radio heap really is, and what the largest single allocation a build requires turns out to be. The host harness in MAV-LUA is not a valid predictor, because it runs under a capped allocator and forcing collection at a tight cap hides accumulated garbage.

Otherwise, the transport boundary remains the sound direction and does not depend on how names are obtained:

1. Keep the bridge generic: accept an explicit set of message IDs and forward payload bytes unchanged, with no autopilot-family interpretation. That is what the reviewed PR already does.
2. Move firmware identity, database/index selection, reply correlation and wire conversion behind MAV-LUA adapters, so PX4 needs no bridge change.
3. Keep ArduPilot readiness polling separate from the generic parameter transport.
4. Add PX4-shaped fixtures proving targeting and raw payloads cross the bridge without an ArduPilot dependency.

Earlier outstanding items, still open:

1. Preserve and review the branch. Inspect Git status including untracked content; `diff --stat` alone omits it.
2. Hardware/RF validation of the half-duplex module and the modified Alpha wiring.
3. Repair the Alpha SD card after the earlier hard lock. Compare `RDY`, `!RDY`, recovered failure, `ARMED` and `RDY?` with Mission Planner; verify the requester works without manual `EXT_STAT` setup and stays silent while status streams.
4. For transport failures, trace 0xAA at the endpoint enqueue, accepted requests entering uartInputBuffer, parsed HEARTBEAT/PARAM_VALUE replies and handset delivery. Capture system/component IDs, index, lengths and timing. Sensor telemetry working alone does not prove the bridge works.
5. Check FIFO pressure and scheduling under live traffic; request drops are currently bounded but silent.
6. On ESP8285 transmitters, measure flash/RAM before proposing upstream.

Do not present this document, a successful build or host tests as radio validation. The next milestone is a bench-tested parameter path.


## Status-frame correction (params-dev4)

The converter incorrectly passed the size of a complete packed frame to
CRSF_FRAME_SIZE in four places: single passthrough, status text, counted
passthrough and flight mode. This adds four unwanted bytes and makes
SetHeaderAndCrc write beyond the frame object. All four now use sizeof(frame.p).
An isolated host probe compiled the actual converter helpers and protocol types:
before the fix, frame lengths were 13/58/20/22 instead of 9/54/16/18; after the
fix all four match their allocated objects. The probe uses a bounds-checking
router double, so it detects the defect without performing the invalid write.

This explains ARM? in MAV-LUA: its strict F2 decoder discarded the overlong
AP-status frame. The params-dev4 Lua accepts exactly four trailing legacy bytes
as well as correctly sized F0/F2 packets, so existing TX firmware can still
provide arm/home status. All received bytes remain validated. Actual status
delivery and arm transitions still need a bench check.

That params-dev4 Lua candidate used explicit `.lua` filenames and source-only
module selection on modern firmware. The current unpackaged loader supersedes that
choice with native SD cache generation as documented above. The loader error, Enter
retry and home layout remain uncommitted. The four frame-size corrections are now
committed on this branch as part of PR #3779.

Do not overwrite the user's modified src/user_defines.txt while building.
