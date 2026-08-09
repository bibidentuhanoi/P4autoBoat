# Wi-Fi Fallback After ESP-NOW LR Probe

## Problem

When the boat does not hear an S3 ground-station beacon, the P4 falls back to
the configured Wi-Fi network. The C6 associates with the access point, but the
P4 never receives `IP_EVENT_STA_GOT_IP` and reports a timeout.

The LR branch added an `esp_wifi_set_protocol()` call inside `wifi_connect()`
to replace the C6's B/G/N/LR bitmap with B/G/N/AX. On ESP-Hosted, changing the
protocol on the already-started radio emits `WIFI_EVENT_STA_STOP`. No matching
start event reaches the host before the C6 associates, so ESP-Hosted suppresses
the connected event that would bring up the host netif and start DHCP.

## Design

Remove the fallback-time protocol reset from `wifi_connect()`. The ESP-NOW
initialization already enables B, G, and N alongside LR. An ordinary access
point can therefore negotiate one of those standard modes without disabling
LR first.

This change is limited to the no-ground-station fallback path:

- A successful ground-station probe continues using ESP-NOW with LR enabled.
- A failed probe calls `wifi_connect()` without causing a C6 station stop.
- The existing ESP-Hosted station-connected event can bring up the P4 netif,
  start DHCP, and produce `IP_EVENT_STA_GOT_IP`.
- No S3 firmware change is required.

## Alternatives Considered

1. Explicitly stop Wi-Fi, reset the protocol, restart Wi-Fi, and then connect.
   This would restore event ordering but adds more radio transitions and more
   failure states to a boot path that does not need a protocol change.
2. Patch ESP-Hosted to forward a connected event after the unmatched stop.
   This changes managed third-party code and masks the unnecessary transition
   in application code.

Removing the reset is the smallest change and preserves the verified LR path.

## Error Handling

Existing connection timeout and disconnect retry behavior remain unchanged.
This fix prevents the false stopped-netif state; it does not add a second DHCP
recovery mechanism.

## Verification

Add a regression test that inspects the production fallback function and fails
if it calls `esp_wifi_set_protocol()`. Run that test before and after the code
change to demonstrate red/green behavior. Then build the P4 firmware. Hardware
acceptance is a boot without the S3 in which the log reaches `Got IP` instead
of `WiFi connection timed out`.
