#include "services/adsb_client.h"

#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#include <ArduinoJson.h>

#include <cstring>

#include "config.h"

namespace services::adsb {

namespace {

constexpr char kApiBase[] = "https://opendata.adsb.fi/api/v3/lat/";
constexpr float kKmPerNm = 1.852f;
constexpr int kConnectAttemptMs = 200;
constexpr unsigned long kRequestTimeoutMs = 10000;

Aircraft s_aircraft[kMaxAircraft];
size_t s_aircraft_count = 0;
PollFn s_poll_fn = nullptr;

void pollNetwork() {
  if (s_poll_fn != nullptr) {
    s_poll_fn();
  }
}

int performGetWithPoll(HTTPClient& http) {
  http.setConnectTimeout(kConnectAttemptMs);
  const unsigned long deadline = millis() + kRequestTimeoutMs;
  while (millis() < deadline) {
    pollNetwork();
    const int code = http.GET();
    if (code > 0) {
      return code;
    }
    if (code != HTTPC_ERROR_CONNECTION_REFUSED &&
        code != HTTPC_ERROR_NOT_CONNECTED) {
      return code;
    }
    delay(5);
  }
  return HTTPC_ERROR_READ_TIMEOUT;
}

bool readResponseBodyWithPoll(HTTPClient& http, String& payload) {
  WiFiClient* stream = http.getStreamPtr();
  if (stream == nullptr) {
    return false;
  }

  const int content_length = http.getSize();
  if (content_length > 0) {
    payload.reserve(static_cast<unsigned>(content_length + 1));
  }

  uint8_t buffer[512];
  const unsigned long deadline = millis() + kRequestTimeoutMs;
  while (millis() < deadline) {
    pollNetwork();
    const int available = stream->available();
    if (available > 0) {
      const int to_read =
          available > static_cast<int>(sizeof(buffer)) ? static_cast<int>(sizeof(buffer))
                                                       : available;
      const int read_bytes = stream->readBytes(buffer, to_read);
      if (read_bytes > 0) {
        payload.concat(reinterpret_cast<const char*>(buffer),
                       static_cast<unsigned>(read_bytes));
      }
    }
    if (content_length > 0 &&
        static_cast<int>(payload.length()) >= content_length) {
      break;
    }
    if (!http.connected() && stream->available() <= 0) {
      break;
    }
    delay(1);
  }

  return payload.length() > 0;
}

float kmToNauticalMiles(float km) { return km / kKmPerNm; }

bool readJsonFloat(const JsonObject& obj, const char* key, float* out) {
  if (obj[key].is<float>() || obj[key].is<double>() || obj[key].is<int>()) {
    *out = obj[key].as<float>();
    return true;
  }
  return false;
}

float pickNoseHeading(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "true_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "track", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    return v;
  }
  return 0.0f;
}

float pickTrackHeading(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "track", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "true_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "mag_heading", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "dir", &v)) {
    return v;
  }
  return 0.0f;
}

float pickGroundSpeed(const JsonObject& plane) {
  float v = 0.0f;
  if (readJsonFloat(plane, "gs", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "tas", &v)) {
    return v;
  }
  if (readJsonFloat(plane, "ias", &v)) {
    return v;
  }
  return 0.0f;
}

bool isOnGround(const JsonObject& plane) {
  if (!plane["alt_baro"].is<const char*>()) {
    return false;
  }
  return strcmp(plane["alt_baro"].as<const char*>(), "ground") == 0;
}

void copyJsonStringTrimmed(const JsonObject& obj, const char* key, char* out,
                           size_t out_len) {
  out[0] = '\0';
  if (out_len == 0 || !obj[key].is<const char*>()) {
    return;
  }
  const char* s = obj[key].as<const char*>();
  size_t n = strnlen(s, out_len - 1);
  while (n > 0 && s[n - 1] == ' ') {
    --n;
  }
  memcpy(out, s, n);
  out[n] = '\0';
}

void formatAltitudeTag(const JsonObject& plane, char* out, size_t out_len) {
  out[0] = '\0';
  if (out_len == 0) {
    return;
  }

  if (plane["alt_baro"].is<const char*>()) {
    const char* s = plane["alt_baro"].as<const char*>();
    if (strcmp(s, "ground") == 0) {
      strncpy(out, "GND", out_len - 1);
      out[out_len - 1] = '\0';
      return;
    }
  }

  float alt = 0.0f;
  if (readJsonFloat(plane, "alt_baro", &alt) ||
      readJsonFloat(plane, "alt_geom", &alt)) {
    snprintf(out, out_len, "%d ft", static_cast<int>(lroundf(alt)));
  }
}

/** Returns true if "flight" (a real callsign, not the hex ICAO fallback) was set. */
bool fillTagFields(Aircraft* ac, const JsonObject& plane) {
  copyJsonStringTrimmed(plane, "flight", ac->callsign, sizeof(ac->callsign));
  const bool has_callsign = ac->callsign[0] != '\0';
  if (!has_callsign) {
    copyJsonStringTrimmed(plane, "hex", ac->callsign, sizeof(ac->callsign));
  }

  copyJsonStringTrimmed(plane, "t", ac->type, sizeof(ac->type));
  formatAltitudeTag(plane, ac->alt, sizeof(ac->alt));
  return has_callsign;
}

// --- Route (origin/destination) lookup ------------------------------------
//
// adsb.fi only reports live position/telemetry, not the flight's route, so
// origin/destination is resolved separately by callsign against adsdb.fi's
// free routes API. Results are cached by callsign so we don't re-query for
// aircraft already resolved, and at most one new lookup is issued per
// fetchUpdate() call so a sky full of traffic can't turn every 3 s poll into
// a burst of blocking HTTPS requests.

constexpr char kRouteApiBase[] = "https://api.adsbdb.com/v0/callsign/";
constexpr size_t kRouteCacheSize = 48;
/** Re-attempt a lookup (success or failure) after this long. */
constexpr unsigned long kRouteCacheTtlMs = 6UL * 60UL * 60UL * 1000UL;  // 6h

struct RouteCacheEntry {
  char callsign[9] = {0};
  char origin[4] = {0};
  char dest[4] = {0};
  bool have_route = false;
  bool in_use = false;
  unsigned long fetched_ms = 0;
};

RouteCacheEntry s_route_cache[kRouteCacheSize];
size_t s_route_cache_next = 0;  // ring-buffer eviction cursor

bool routeCacheIsFresh(const RouteCacheEntry& e) {
  return e.in_use && (millis() - e.fetched_ms) < kRouteCacheTtlMs;
}

RouteCacheEntry* findRouteCacheEntry(const char* callsign) {
  for (size_t i = 0; i < kRouteCacheSize; ++i) {
    if (s_route_cache[i].in_use &&
        strncmp(s_route_cache[i].callsign, callsign, sizeof(RouteCacheEntry::callsign)) == 0) {
      return &s_route_cache[i];
    }
  }
  return nullptr;
}

RouteCacheEntry* claimRouteCacheEntry(const char* callsign) {
  RouteCacheEntry* e = findRouteCacheEntry(callsign);
  if (e == nullptr) {
    e = &s_route_cache[s_route_cache_next];
    s_route_cache_next = (s_route_cache_next + 1) % kRouteCacheSize;
  }
  strncpy(e->callsign, callsign, sizeof(e->callsign) - 1);
  e->callsign[sizeof(e->callsign) - 1] = '\0';
  e->origin[0] = '\0';
  e->dest[0] = '\0';
  e->have_route = false;
  e->in_use = true;
  e->fetched_ms = millis();
  return e;
}

/** Looks up one callsign's route via adsdb.fi and stores it in the cache. */
void fetchRouteForCallsign(const char* callsign) {
  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  const String url = String(kRouteApiBase) + callsign;
  if (!http.begin(client, url)) {
    Serial.println("route: http.begin failed");
    return;
  }

  http.setTimeout(kRequestTimeoutMs);
  const int code = performGetWithPoll(http);
  if (code != HTTP_CODE_OK) {
    Serial.printf("route: HTTP %d for %s\n", code, callsign);
    http.end();
    claimRouteCacheEntry(callsign);  // cache the miss so we back off for the TTL
    return;
  }

  String payload;
  if (!readResponseBodyWithPoll(http, payload)) {
    Serial.println("route: empty response");
    http.end();
    claimRouteCacheEntry(callsign);
    return;
  }
  http.end();

  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, payload);
  RouteCacheEntry* e = claimRouteCacheEntry(callsign);
  if (err) {
    Serial.printf("route: JSON parse error: %s\n", err.c_str());
    return;
  }

  // Unknown callsigns come back as {"response":"unknown callsign"} (a string,
  // not an object), so bail out unless we actually got a flightroute object.
  JsonObject flightroute = doc["response"]["flightroute"].as<JsonObject>();
  if (flightroute.isNull()) {
    return;
  }

  const char* origin_iata = flightroute["origin"]["iata_code"] | "";
  const char* origin_icao = flightroute["origin"]["icao_code"] | "";
  const char* dest_iata = flightroute["destination"]["iata_code"] | "";
  const char* dest_icao = flightroute["destination"]["icao_code"] | "";

  const char* origin = origin_iata[0] != '\0' ? origin_iata : origin_icao;
  const char* dest = dest_iata[0] != '\0' ? dest_iata : dest_icao;
  if (origin[0] == '\0' || dest[0] == '\0') {
    return;
  }

  strncpy(e->origin, origin, sizeof(e->origin) - 1);
  e->origin[sizeof(e->origin) - 1] = '\0';
  strncpy(e->dest, dest, sizeof(e->dest) - 1);
  e->dest[sizeof(e->dest) - 1] = '\0';
  e->have_route = true;
  Serial.printf("route: %s -> %s (%s)\n", e->origin, e->dest, callsign);
}

/** Fills ac->origin/dest from the cache; leaves them empty if not yet resolved. */
void applyCachedRoute(Aircraft* ac, bool has_callsign) {
  ac->origin[0] = '\0';
  ac->dest[0] = '\0';
  if (!has_callsign) {
    return;
  }
  const RouteCacheEntry* e = findRouteCacheEntry(ac->callsign);
  if (e != nullptr && e->have_route) {
    strncpy(ac->origin, e->origin, sizeof(ac->origin) - 1);
    ac->origin[sizeof(ac->origin) - 1] = '\0';
    strncpy(ac->dest, e->dest, sizeof(ac->dest) - 1);
    ac->dest[sizeof(ac->dest) - 1] = '\0';
  }
}

/** Issues at most one new route lookup for the current aircraft list. */
void resolveOneRoute(const Aircraft* aircraft, const bool* has_callsign, size_t n) {
  for (size_t i = 0; i < n; ++i) {
    if (!has_callsign[i]) {
      continue;
    }
    const RouteCacheEntry* e = findRouteCacheEntry(aircraft[i].callsign);
    if (e != nullptr && routeCacheIsFresh(*e)) {
      continue;  // resolved (or a recent miss) already
    }
    fetchRouteForCallsign(aircraft[i].callsign);
    return;  // one HTTPS round-trip per fetchUpdate() is enough
  }
}

}  // namespace

void setPollFn(PollFn fn) { s_poll_fn = fn; }

size_t aircraftCount() { return s_aircraft_count; }

const Aircraft* aircraftList() { return s_aircraft; }

bool fetchUpdate(double center_lat, double center_lon, float fetch_radius_km) {
  const float dist_nm = kmToNauticalMiles(fetch_radius_km);

  String url = kApiBase;
  url += String(center_lat, 6);
  url += "/lon/";
  url += String(center_lon, 6);
  url += "/dist/";
  url += String(dist_nm, 1);

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  if (!http.begin(client, url)) {
    Serial.println("adsb: http.begin failed");
    return false;
  }

  http.setTimeout(kRequestTimeoutMs);
  const int code = performGetWithPoll(http);
  if (code != HTTP_CODE_OK) {
    Serial.printf("adsb: HTTP %d\n", code);
    http.end();
    return false;
  }

  String payload;
  if (!readResponseBodyWithPoll(http, payload)) {
    Serial.println("adsb: empty response");
    http.end();
    return false;
  }
  http.end();

  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, payload);
  if (err) {
    Serial.printf("adsb: JSON parse error: %s\n", err.c_str());
    return false;
  }

  JsonArray ac = doc["ac"].as<JsonArray>();
  if (ac.isNull()) {
    s_aircraft_count = 0;
    return true;
  }

  size_t n = 0;
  bool has_callsign[kMaxAircraft];
  for (JsonObject plane : ac) {
    if (n >= kMaxAircraft) {
      break;
    }
    if (!plane["lat"].is<float>() || !plane["lon"].is<float>()) {
      continue;
    }
    if (isOnGround(plane) && !config::kAdsbShowGroundAircraft) {
      continue;
    }

    s_aircraft[n].lat = plane["lat"].as<float>();
    s_aircraft[n].lon = plane["lon"].as<float>();
    s_aircraft[n].nose_deg = pickNoseHeading(plane);
    s_aircraft[n].track_deg = pickTrackHeading(plane);
    s_aircraft[n].gs_knots = pickGroundSpeed(plane);
    has_callsign[n] = fillTagFields(&s_aircraft[n], plane);
    applyCachedRoute(&s_aircraft[n], has_callsign[n]);
    ++n;
  }

  s_aircraft_count = n;
  Serial.printf("adsb: %u aircraft\n", static_cast<unsigned>(n));

  // Backfill one route per cycle; already-cached routes were applied above.
  resolveOneRoute(s_aircraft, has_callsign, n);
  return true;
}

}  // namespace services::adsb
